#include <linux/netlink.h>
#include <net/sock.h>

#include "h264e_rc.h"
#include "h264e_rc_nl.h"




static struct sock *nl_sk;
static DEFINE_MUTEX(netlink_mutex);
static int h264e_rc_server_online = 0;

//static DECLARE_COMPLETION(msg_complete);

static int h264e_netlink_send_msg(struct h264e_ctx *ctx, void *buf, unsigned int len)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh;
	int ret = 0;
	int timeout = 0;


	skb = nlmsg_new(len, GFP_ATOMIC);
	if(!skb) {
		pr_err("nlmsg_new error!\n");
		return -ENOMEM;
	}

	nlh = nlmsg_put(skb, ctx->pid, 0, 0, len, 0);
	if(!nlh) {
		kfree(skb);
		pr_err("nlmsg_put error!\n");
		return -EINVAL;
	}

	memcpy(nlmsg_data(nlh), buf, len);


	/*wait for completion.*/

	reinit_completion(&ctx->msg_complete);

	ret = netlink_unicast(nl_sk, skb, ctx->pid, 0);

	timeout = wait_for_completion_timeout(&ctx->msg_complete, msecs_to_jiffies(30));
	if(!timeout) {
		printk("h264e nl send msg timeout!\n");
		return -ETIMEDOUT;
	}

#if 0
	//return netlink_unicast(nl_sk, skb, pid, MSG_DONTWAIT);
	while(!wait_msg_recv) {
		msleep(1);
	}
#endif

	return ret;
}

static int h264e_netlink_rcv_msg(struct sk_buff *skb, struct nlmsghdr *nlh, struct netlink_ext_ack *extack)
{

	struct h264e_nl_msg *msg = NLMSG_DATA(nlh);
	struct h264e_ctx *ctx = NULL;

	switch (nlh->nlmsg_type) {
		default:
		//	pr_err("unknown nlmsg_type\n");
			break;
	}

	switch (msg->cmd) {
		case VPU_CMD_SERVER_ONLINE:
			h264e_rc_server_online = 1;
			break;
		case VPU_CMD_SERVER_OFFLINE:
			h264e_rc_server_online = 0;
			break;

		default:
			break;
	}

	ctx = (struct h264e_ctx *)msg->ctx_id;

	if(ctx) {
		if((nlh->nlmsg_len - NLMSG_HDRLEN) > H264E_MAX_NL_MSG) {
			printk("recved msg too long: %d, max: %d\n", nlh->nlmsg_len, H264E_MAX_NL_MSG);
		} else {
			memcpy(ctx->recved_msg, NLMSG_DATA(nlh), nlh->nlmsg_len - NLMSG_HDRLEN);
		}

		complete(&ctx->msg_complete);
	}


	return 0;
}

static void h264e_netlink_rcv(struct sk_buff *skb)
{
	mutex_lock(&netlink_mutex);
	netlink_rcv_skb(skb, &h264e_netlink_rcv_msg);
	mutex_unlock(&netlink_mutex);
}

static int start_h264e_server(void)
{
#if 0
	const char *server = "/usr/bin/h264e-nl-server"
	static const char *argv[MAX_INIT_ARGS+2] = { "/usr/bin/h264e-nl-server", NULL, };
#endif

	if(h264e_rc_server_online) {
		return 0;
	}


#if 0
	return do_execve(getname_kernel(),
        (const char __user *const __user *)argv_init,
        (const char __user *const __user *)envp_init);
#endif

	/*kernel start h264e_rc_server.*/

	/*wait for h264e_rc_server_online = 1.*/
}

int h264e_netlink_test(void)
{
	struct netlink_kernel_cfg nl_cfg = {
		.input  = h264e_netlink_rcv,
	};


	nl_sk = netlink_kernel_create(&init_net, NETLINK_H264_ENCODER, &nl_cfg);
	if(!nl_sk) {
		printk("failed to create netlink\n");
		return -EINVAL;
	}

	return 0;
}


/*调用远程的创建上下文.*/
int h264e_nl_init_ctx(struct h264e_ctx *ctx, struct nl_ctx_info *info)
{
	/*远端主要处理的内容.*/
	int id = 0;
	unsigned int msg_len = sizeof(struct h264e_nl_msg) + sizeof(struct nl_ctx_info);

	if(!h264e_rc_server_online) {
		printk("h264e server not online\n");
		return -EINVAL;
	}

	struct h264e_nl_msg *msg = kmalloc(msg_len, GFP_KERNEL);

#if 0
	if(!netlink_has_listeners(nl_sk, 1)) {
		printk("h264e netlink has no listeners, start server first!\n");
		return -EINVAL;
	}
#endif

	ctx->pid = NETLINK_H264_PORT;
	init_completion(&ctx->msg_complete);

	msg->ctx_id = ctx;	/*以ctx的地址作为ctx_id*/
	msg->cmd = VPU_CMD_CREATE_CTX;
	memcpy(msg->buf, info, sizeof(struct nl_ctx_info));

	h264e_netlink_send_msg(ctx, msg, msg_len);


	/*wait done.*/

	kfree(msg);

	/*return values. */
	msg = (struct h264e_nl_msg *)ctx->recved_msg;

	return msg->ret_val;
}

/*释放远端的上下文句柄.*/
int h264e_nl_free_ctx(struct h264e_ctx *ctx)
{

	struct h264e_nl_msg msg;


	msg.ctx_id = ctx;
	msg.cmd = VPU_CMD_FREE_CTX;

	h264e_netlink_send_msg(ctx, &msg, sizeof(struct h264e_nl_msg));

	return 0;
}

int h264e_nl_setup_headers(struct h264e_ctx *ctx, struct nl_header_info *info)
{
	unsigned int msg_len = sizeof(struct h264e_nl_msg) + sizeof(struct nl_header_info);
	struct h264e_nl_msg *msg = NULL;

	if(!h264e_rc_server_online) {
		printk("%s, %d h264e server not online\n", __func__, __LINE__);
		return -EINVAL;
	}


	msg = kmalloc(msg_len, GFP_KERNEL);
	msg->ctx_id = ctx;
	msg->cmd = VPU_CMD_SETUP_HEADERS;


	memcpy(msg->buf, info, sizeof(struct nl_header_info));

	h264e_netlink_send_msg(ctx, msg, msg_len);

	kfree(msg);


	return 0;
}

int h264e_nl_rc_video_cfg(struct h264e_ctx *ctx, struct h264e_params *p)
{
	unsigned int msg_len = sizeof(struct h264e_nl_msg) + sizeof(struct h264e_params);
	struct h264e_nl_msg *msg = kmalloc(msg_len, GFP_KERNEL);

	msg->ctx_id = ctx;
	msg->cmd = VPU_CMD_RC_VIDEO_CFG;

	memcpy(msg->buf, p, sizeof(struct h264e_params));

	h264e_netlink_send_msg(ctx, msg, msg_len);


	kfree(msg);
	return 0;
}

int h264e_nl_rc_frame_start(struct h264e_ctx *ctx, struct h264e_frame_start_params *p)
{
	unsigned int msg_len = sizeof(struct h264e_nl_msg) + sizeof(struct h264e_frame_start_params);
	struct h264e_nl_msg *msg = kmalloc(msg_len, GFP_KERNEL);

	msg->ctx_id = ctx;
	msg->cmd = VPU_CMD_RC_FRAME_START;

	memcpy(msg->buf, p, sizeof(struct h264e_frame_start_params));
	h264e_netlink_send_msg(ctx, msg, msg_len);
	/*wait.*/

	kfree(msg);

	/*return values. */
	msg = (struct h264e_nl_msg *)ctx->recved_msg;
	memcpy(p, msg->buf, sizeof(struct h264e_frame_start_params));

	return 0;
}

int h264e_nl_rc_frame_end(struct h264e_ctx *ctx, struct h264e_frame_end_params *p)
{
	unsigned int msg_len = sizeof(struct h264e_nl_msg) + sizeof(struct h264e_frame_end_params);
	struct h264e_nl_msg *msg = kmalloc(msg_len, GFP_KERNEL);

	msg->ctx_id = ctx;
	msg->cmd = VPU_CMD_RC_FRAME_END;

	memcpy(msg->buf, p, sizeof(struct h264e_frame_end_params));
	h264e_netlink_send_msg(ctx, msg, msg_len);

	/*wait.*/

	kfree(msg);
	return 0;
}

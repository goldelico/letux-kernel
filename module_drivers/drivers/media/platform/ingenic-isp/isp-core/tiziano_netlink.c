#include <linux/init.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/types.h>
#include <net/sock.h>
#include <net/netlink.h>
#include "inc/tiziano_netlink.h"

#define NETLINK_USER	22
#define USER_MSG		(NETLINK_USER + 1)
#define USER_PORT		50

#if 0
static struct sock *nlsk = NULL;
#endif
struct nl_sock_map {
	tisp_netlink_t *nl;
	struct sock *nlsk;
};

#define MAX_NL_SK	2
struct nl_sock_map nl_sock_map[MAX_NL_SK] = {NULL};

static tisp_netlink_t * sock_to_nl(struct sock *sk)
{
	int i = 0;
	for(i = 0; i < MAX_NL_SK; i++) {
		if(nl_sock_map[i].nlsk == sk) {
			return nl_sock_map[i].nl;
		}
	}

	return NULL;
}

//net_event_cb net_event_process = NULL;

int32_t netlink_send_msg(tisp_netlink_t *nl, int8_t *pbuf, uint16_t len)
{
	int32_t ret;
	struct sk_buff *nl_skb;
	struct nlmsghdr *nlh;
	nl_skb = nlmsg_new(len, GFP_ATOMIC);
	if (!nl_skb) {
		pr_err("%s,%d: netlink_alloc_skb error\n", __func__, __LINE__);
		return -1;
	}
	nlh = nlmsg_put(nl_skb, 0, 0, USER_MSG, len, 0);
	if (nlh == NULL) {
		pr_err("%s,%d: nlmsg_put() error\n", __func__, __LINE__);
		nlmsg_free(nl_skb);
		return -1;
	}
	memcpy(nlmsg_data(nlh), pbuf, len);
	ret = netlink_unicast(nl->nlsk, nl_skb, USER_PORT, MSG_DONTWAIT);
	return ret;
}

static void netlink_rcv_msg(struct sk_buff *skb)
{
	struct nlmsghdr *nlh = NULL;
	struct sock *sock = skb->sk;
	tisp_netlink_t * nl = sock_to_nl(sock);
	void *data = NULL;

	if (skb->len >= nlmsg_total_size(0)) {
		nlh = nlmsg_hdr(skb);
		data = NLMSG_DATA(nlh);
		if (data) {
			if (NULL != nl->net_event_process) {
				nl->net_event_process(nl->cb_data, data, nlmsg_len(nlh));
			}
			//netlink_send_msg(data, nlmsg_len(nlh));
		}
	}
}

struct netlink_kernel_cfg nlcfg = {
	.input  = netlink_rcv_msg,
};

int32_t tisp_netlink_event_set_cb(tisp_netlink_t *nl, net_event_cb cb, void *cb_data)
{
	nl->net_event_process = cb;
	nl->cb_data = cb_data;
	return 0;
}

int32_t tisp_netlink_init(tisp_netlink_t *nl)
{
//	pr_err("%s,%d: \n", __func__, __LINE__);
	int i;
	nl->nlsk = (struct sock *)netlink_kernel_create(&init_net, USER_MSG, &nlcfg);
	if (!nl->nlsk) {
		pr_err("%s,%d: create netlink socket error.\n", __func__, __LINE__);
		return -1;
	}

	for(i = 0; i < MAX_NL_SK; i++) {
		if(nl_sock_map[i].nl == NULL || nl_sock_map[i].nlsk == NULL) {
			nl_sock_map[i].nl = nl;
			nl_sock_map[i].nlsk = nl->nlsk;
		}
	}
//	pr_err("%s,%d: \n", __func__, __LINE__);
	return 0;
}

void tisp_netlink_exit(tisp_netlink_t *nl)
{
	int i;

	if (nl->nlsk != NULL && nl->nlsk->sk_socket != NULL) {
		sock_release(nl->nlsk->sk_socket);
	}

	for(i = 0; i < MAX_NL_SK; i++) {
		if(nl_sock_map[i].nl == nl) {
			nl_sock_map[i].nl = NULL;
			nl_sock_map[i].nlsk = NULL;
		}
	}

}

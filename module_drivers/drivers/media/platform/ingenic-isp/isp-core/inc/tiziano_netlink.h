#ifndef __TIZIANO_NETLINK_H__
#define __TIZIANO_NETLINK_H__
#include "tiziano_core.h"

int32_t netlink_send_msg(tisp_netlink_t *nl, int8_t *pbuf, uint16_t len);
int32_t tisp_netlink_event_set_cb(tisp_netlink_t *nl, net_event_cb cb, void *cb_data);

int32_t tisp_netlink_init(tisp_netlink_t *nl);

void tisp_netlink_exit(tisp_netlink_t *nl);

#endif

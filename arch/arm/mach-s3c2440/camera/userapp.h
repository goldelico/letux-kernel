/*
    Character Driver API Interface

    Copyright (C) 2003 Samsung Electronics (SW.LEE: hitchcar@samsung.com)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

*/

#ifndef __FIMC20_CAMIF_USR_APP_H_
#define __FIMC20_CAMIF_USR_APP_H_


/* 
 * IOCTL Command for Character Driver
 */ 

#define CMD_CAMERA_INIT   0x23
/*  Test Application Usage */
typedef struct {
        int src_x;
        int src_y;
        int dst_x;
        int dst_y;
        int bpp;
        int flip;
} camif_param_t;



#endif


/* 
 * Local variables:
 * tab-width: 8
 *  c-indent-level: 8
 *  c-basic-offset: 8
 *  c-set-style: "K&R"
 * End:
 */

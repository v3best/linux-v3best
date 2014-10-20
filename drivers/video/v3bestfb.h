/*
 * Platform device data for V3best Framebuffer device
 *
 * Copyright 2014 V3best Ltd.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef __V3BESTFB_H__
#define __V3BESTFB_H__

#include <linux/types.h>

/* Zing-Snowleo reference design framebuffer driver platform data struct */
struct v3bestfb_platform_data {
    u32 rotate_screen;	/* Flag to rotate display 180 degrees */
    u32 screen_height_mm;	/* Physical dimensions of screen in mm */
    u32 screen_width_mm;
    u32 xres, yres;		/* resolution of screen in pixels */
    u32 xvirt, yvirt;	/* resolution of memory buffer */

    /* Physical address of framebuffer memory; If non-zero, driver
     * will use provided memory address instead of allocating one from
     * the consistent pool. */
    u32 fb_phys;
};
struct v3bestfb_vmode_data {
    u32 refresh;
    u32 h_active;
    u32 v_active;
    u32 pixclock;
    u32 hfp;
    u32 hbp;
    u32 vfp;
    u32 vbp;
    u32 hsync_len;
    u32 vsync_len;
};

#endif  /* __V3BESTFB_H__ */

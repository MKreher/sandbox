/*
 * Copyright (c) 2018 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/display.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <zephyr.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(app);

static lv_disp_draw_buf_t disp_buf;
static lv_color_t buf_1[200 * 200]; // full screen size buffer
static lv_color_t buf_2 = NULL; // full screen size buffer

static lv_disp_drv_t disp_drv;

static struct display_buffer_descriptor buf_desc;


void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
        LOG_INF("my_flush_cb() called.");

        /* IMPORTANT!!!
         * Inform the graphics library that you are ready with the flushing*/
        lv_disp_flush_ready(disp_drv);
}

void my_monitor_cb(lv_disp_drv_t * disp_drv, uint32_t time, uint32_t px)
{
        LOG_INF("my_monitor_cb() called.");
}

void main(void)
{
	const struct device *display_dev;

	display_dev = device_get_binding(CONFIG_LVGL_DISPLAY_DEV_NAME);

	if (display_dev == NULL) {
		LOG_ERR("device not found.  Aborting test.");
		return;
	}

        lv_init();
        lv_disp_draw_buf_init(&disp_buf, buf_1, buf2, 200 * 200);

        
        lv_disp_drv_init(&disp_drv);            /*Basic initialization*/

	buf_desc.buf_size = 200 * 200 / 8;
	buf_desc.width = 200;
	buf_desc.height = 200;

        disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
        disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
        disp_drv.hor_res = 200;                 /*Set the horizontal resolution in pixels*/
        disp_drv.ver_res = 200;                 /*Set the vertical resolution in pixels*/

        lv_disp_t * disp;
        disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/        
        
        // Display Hello World
        lv_obj_t *hello_world_label;
        hello_world_label = lv_label_create(lv_scr_act(), NULL);
        lv_label_set_text(hello_world_label, "Hello world!");
        lv_obj_align(hello_world_label, NULL, LV_ALIGN_CENTER, 0, 0);
        //lv_label_set_long_mode(hello_world_label, LV_LABEL_LONG_CROP);
        lv_obj_set_width(hello_world_label, 100);
        lv_label_set_align(hello_world_label, LV_LABEL_ALIGN_LEFT);

	lv_task_handler();
}
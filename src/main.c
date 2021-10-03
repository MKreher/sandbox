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

void my_flush_cb(struct _disp_drv_t *disp_drv,
		const lv_area_t *area, lv_color_t *color_p)
{
	uint16_t w = area->x2 - area->x1 + 1;
	uint16_t h = area->y2 - area->y1 + 1;
	const struct device *display_dev = (const struct device *)disp_drv->user_data;
	struct display_capabilities cap;
	struct display_buffer_descriptor desc;

	display_get_capabilities(display_dev, &cap);

	desc.buf_size = (w * h)/8U;
	desc.width = w;
	desc.pitch = w;
	desc.height = h;

	display_write(display_dev, area->x1, area->y1, &desc, color_p);

	lv_disp_flush_ready(disp_drv);
}

LV_FONT_DECLARE(font_jet_brains_mono_bold_18);
LV_FONT_DECLARE(font_space_mono_regular_18);
LV_IMG_DECLARE(tictac);

void main(void)
{       
        LOG_INF("main() called.");

	lv_disp_t *lv_disp = lv_disp_get_default();
        lv_disp_drv_t lv_disp_drv = lv_disp->driver;

        lv_disp_drv.flush_cb   = my_flush_cb;

        lv_disp_drv_update(lv_disp, &lv_disp_drv);
        //static lv_style_t my_style;
        //lv_style_init(&my_style);
        //lv_style_set_text_font(&my_style, LV_STATE_DEFAULT, LV_FONT_MONTSERRAT_12);
        
        // Display Hello World
        lv_obj_t *hello_world_label;
        hello_world_label = lv_label_create(lv_scr_act(), NULL);
        //lv_obj_add_style(hello_world_label, 0, &my_style);
        //lv_obj_set_style_local_text_font(hello_world_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_18);
        lv_obj_set_style_local_text_font(hello_world_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &font_space_mono_regular_18);
        //lv_obj_report_style_mod(NULL);
        lv_obj_refresh_style(hello_world_label, LV_LABEL_PART_MAIN, LV_STYLE_PROP_ALL);
        //lv_label_set_text(hello_world_label, "Hallo Michael!");
        //lv_label_set_text(hello_world_label, LV_SYMBOL_OK " " "Hallo Michael!");
        lv_label_set_text(hello_world_label, "ÄÖÜäöüßá´`'#.,|");
        lv_obj_align(hello_world_label, NULL, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_width(hello_world_label, 200);
        lv_label_set_align(hello_world_label, LV_LABEL_ALIGN_LEFT);
        
        // display image
        /*
        lv_obj_t *tictac_img;
        tictac_img = lv_img_create(lv_scr_act(), NULL);
        lv_img_set_src(tictac_img,  &tictac);
        lv_obj_align(tictac_img, NULL, LV_ALIGN_CENTER, 0, 0);
        */
        

	lv_task_handler();
}
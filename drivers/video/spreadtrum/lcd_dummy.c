/* 
 * Dummy LCD Panel
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/err.h>
#include "lcdpanel.h"
//#define  LCD_DEBUG
#ifdef LCD_DEBUG
#define LCD_PRINT printk
#else
#define LCD_PRINT(...)
#endif

static int32_t dummy_init(struct panel_spec *self)
{
	return 0;
}

static int32_t dummy_set_window(struct panel_spec *self,
		uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
	return 0;
}

static int32_t dummy_invalidate(struct panel_spec *self)
{
	return self->ops->panel_set_window(self, 0, 0, 
			self->width - 1, self->height - 1);
	
}

static int32_t dummy_invalidate_rect(struct panel_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom)
{
	return self->ops->panel_set_window(self, left, top, 
			right, bottom);
}


static int32_t dummy_set_direction(struct panel_spec *self, uint16_t direction)
{
	return 0;
}

static int32_t dummy_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	return 0;
}
static uint32_t dummy_read_id(struct panel_spec *self)
{

	return 0x57;
}
static struct panel_operations lcd_dummy_operations = {
	.panel_init = dummy_init,
	.panel_set_window = dummy_set_window,
	.panel_invalidate_rect= dummy_invalidate_rect,
	.panel_invalidate = dummy_invalidate,
	.panel_set_direction = dummy_set_direction,
	.panel_enter_sleep = dummy_enter_sleep,
	.panel_readid          = dummy_read_id,
};

static struct timing_mcu lcd_dummy_timing[] = {
[LCD_REGISTER_TIMING] = {                    // read/write register timing
		.rcss = 15,  // 15ns
		.rlpw = 60,
		.rhpw = 60,
		.wcss = 10,
		.wlpw = 35,
		.whpw = 35,
	},
[LCD_GRAM_TIMING] = {                    // read/write gram timing
		.rcss = 15,  // 15ns
		.rlpw = 60,
		.rhpw = 60,
		.wcss = 10,
		.wlpw = 35,
		.whpw = 35,
	},
};

static struct info_mcu lcd_dummy_info = {
	.bus_mode = LCD_BUS_8080,
	.bus_width = 18,
	.timing = lcd_dummy_timing,
	.ops = NULL,
};

struct panel_spec lcd_panel_dummy = {
	.width = 480,
	.height = 800,
	.mode = LCD_MODE_MCU,
	.direction = LCD_DIRECT_NORMAL,
	.info = {.mcu = &lcd_dummy_info},
	.ops = &lcd_dummy_operations,
};

struct panel_cfg lcd_dummy = {
	/* this panel may on both CS0/1 */
	.lcd_cs = -1,
	.lcd_id = 0x57,
	.lcd_name = "lcd_dummy",
	.panel = &lcd_panel_dummy,
};





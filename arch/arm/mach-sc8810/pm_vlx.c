/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <asm/irqflags.h>
#include <mach/pm_debug.h>

extern int sc8810_deep_sleep(void);
extern void sc8810_pm_init(void);

/*for battery*/
#define BATTERY_CHECK_INTERVAL 30000
extern int get_batt_percentage();
//extern void battery_sleep(void);

static int sprd_check_battery(void)
{
        int ret_val = 0;
        if (get_batt_percentage()) {
                ret_val = 1;
        }
        return ret_val;
}

static void sc8810_sleep(void){
	cpu_do_idle();
}

static int pm_deepsleep(suspend_state_t state)
{
	int ret_val = 0;
	unsigned long flags;
	u32 battery_time, cur_time;
	battery_time = cur_time = get_sys_cnt();
	/* add for debug & statisic*/
	clr_sleep_mode();
	time_statisic_begin();

	while(1){
		hw_local_irq_disable();
		local_irq_save(flags);
		if (arch_local_irq_pending()) {
			/* add for debug & statisic*/
			irq_wakeup_set();

			local_irq_restore(flags);
			hw_local_irq_enable();
			break;
		}else{
			local_irq_restore(flags);
			WARN_ONCE(!irqs_disabled(), "#####: Interrupts enabled in pm_enter()!\n");

			ret_val = os_ctx->idle(os_ctx);
			if (0 == ret_val) {
				sc8810_deep_sleep();
			}
			hw_local_irq_enable();
		}

		//battery_sleep();
		cur_time = get_sys_cnt();
		if ((cur_time -  battery_time) > BATTERY_CHECK_INTERVAL) {
			battery_time = cur_time;
			if (sprd_check_battery()) {
				printk("###: battery low!\n");
				break;
			}
		}
	}
	/* add for debug & statisic*/
	time_statisic_end();

	return ret_val;
}


static int sc8810_pm_enter(suspend_state_t state)
{
	int rval = 0;
	switch (state) {
		case PM_SUSPEND_STANDBY:
			sc8810_sleep();
			break;
		case PM_SUSPEND_MEM:
			rval = pm_deepsleep(state);
			break;
		default:
			break;
	}

	return rval;
}

static int sc8810_pm_valid(suspend_state_t state)
{
	pr_debug("pm_valid: %d\n", state);
	switch (state) {
		case PM_SUSPEND_ON:
		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
			return 1;
		default:
			return 0;
	}
}

extern void check_ldo(void);
extern void check_pd(void);
static int sc8810_pm_prepare(void)
{
	pr_debug("enter %s\n", __func__);
	check_ldo();
	check_pd();
	return 0;
}

static void sc8810_pm_finish(void)
{
	pr_debug("enter %s\n", __func__);
	print_statisic();
}

static struct platform_suspend_ops sc8810_pm_ops = {
	.valid		= sc8810_pm_valid,
	.enter		= sc8810_pm_enter,
	.prepare		= sc8810_pm_prepare,
	.prepare_late 	= NULL,
	.finish		= sc8810_pm_finish,
};

static int __init pm_init(void)
{
	sc8810_pm_init();

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&sc8810_pm_ops);
#endif

	return 0;
}

device_initcall(pm_init);

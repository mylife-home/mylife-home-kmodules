/* Copyright (C) 2014 Vincent TRUMPFF
 *
 * May be copied or modified under the terms of the GNU General Public
 * License. See linux/COPYING for more information.
 *
 * Generic software-only driver for AC zero crossing detector
 * via GPIO lib interface.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/math64.h>
#include <linux/string.h>

#include "ac_common.h"
#include "ac_zc.h"

#define get_ktime_secs(ktime) (div_s64((ktime).tv64, NSEC_PER_SEC))
#define get_now_secs() get_ktime_secs(ktime_get())

static int ac_zc_gpio = -1;
static int ac_zc_irq = -1;

static int ac_zc_freq_value = 0;
static int ac_zc_freq_counter = 0;
static s64 ac_zc_freq_start;
static int ac_zc_gpio_previous_value;

struct ac_zc_cb_desc
{
	int status; // 0 = disabled
	void *cb_data;
	ac_zc_callback cb;
};

// TODO : resizable list ?
#define ZC_DESCRIPTOR_SIZE 16
static struct ac_zc_cb_desc zc_descriptors[ZC_DESCRIPTOR_SIZE];

// lock protects against ac_zc_register() / ac_zc_unregister()
static DEFINE_MUTEX(ac_zc_descriptors_lock);

static ssize_t ac_zc_attr_show(struct class *class, struct class_attribute *attr, char *buf);
static irqreturn_t ac_zc_irq_handler(int irq, void *dev_id);
static int ac_zc_init(void);
static void ac_zc_exit(void);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vincent TRUMPFF");
MODULE_DESCRIPTION("Driver for AC zero crossing detector");

module_param(ac_zc_gpio, int, 0444);
MODULE_PARM_DESC(ac_zc_gpio, "Zero crossing detector GPIO number");

EXPORT_SYMBOL(ac_zc_register);
EXPORT_SYMBOL(ac_zc_unregister);
EXPORT_SYMBOL(ac_zc_freq);

module_init(ac_zc_init);
module_exit(ac_zc_exit);

// return : id > 0 on success (to unregister), error < 0 on failure
int ac_zc_register(int status, ac_zc_callback cb, void *cb_data)
{
	int ret;
	unsigned int index;
	struct ac_zc_cb_desc *desc;

	if(status <= 0 || status > (AC_ZC_STATUS_ENTER | AC_ZC_STATUS_LEAVE))
		return -EINVAL;
	if(!cb)
		return -EINVAL;

	mutex_lock(&ac_zc_descriptors_lock);

	ret = -EBUSY; // no empty place in array
	for(index = 0; index < ZC_DESCRIPTOR_SIZE; ++index)
	{
		desc = zc_descriptors + index;
		if(desc->status)
			continue;

		desc->cb = cb;
		desc->cb_data = cb_data;
		desc->status = status;

		ret = index+1;
		break;
	}

	mutex_unlock(&ac_zc_descriptors_lock);

	return ret;
}

int ac_zc_unregister(int id)
{
	if(id <= 0 || id > ZC_DESCRIPTOR_SIZE)
		return -EINVAL;

	mutex_lock(&ac_zc_descriptors_lock);

	zc_descriptors[id-1].status = 0;

	mutex_unlock(&ac_zc_descriptors_lock);

	return 0;
}

int ac_zc_freq(void)
{
	return ac_zc_freq_value;
}

// Sysfs definitions for ac_zc class
static struct class_attribute ac_zc_class_attrs[] =
{
	__ATTR(gpio, 0444, ac_zc_attr_show, NULL),
	__ATTR(freq, 0444, ac_zc_attr_show, NULL),
	__ATTR_NULL,
};

static struct class ac_zc_class =
{
	.name =        "ac_zc",
	.owner =       THIS_MODULE,
	.class_attrs = ac_zc_class_attrs,
};

// Show attributes values for zero crossing detector
ssize_t ac_zc_attr_show(struct class *class, struct class_attribute *attr, char *buf)
{
	ssize_t status;

	if(strcmp(attr->attr.name, "gpio") == 0)
		status = sprintf(buf, "%d\n", ac_zc_gpio);
	else if(strcmp(attr->attr.name, "freq") == 0)
		status = sprintf(buf, "%d Hz\n", (ac_zc_freq_start == get_now_secs()) ? ac_zc_freq_value : 0);
	else
		status = -EIO;

	return status;
}

irqreturn_t ac_zc_irq_handler(int irq, void *dev_id)
{
	s64 now_secs;
	int gpio_value;
	struct ac_zc_cb_desc *desc;
	int index;
	int status;

	if(irq != ac_zc_irq)
		return IRQ_NONE;
	if(dev_id != &ac_zc_class)
		return IRQ_NONE;

	gpio_value = gpio_get_value(ac_zc_gpio);
	if(gpio_value == ac_zc_gpio_previous_value)
		return IRQ_HANDLED;
	ac_zc_gpio_previous_value = gpio_value;

	//callbacks
	status = gpio_value ? AC_ZC_STATUS_ENTER : AC_ZC_STATUS_LEAVE;
	for(index=0; index<ZC_DESCRIPTOR_SIZE; ++index)
	{
		desc = zc_descriptors + index;
		if(desc->status & status)
			desc->cb(status, desc->cb_data);
	}

	// stats
	if(!gpio_value)
		return IRQ_HANDLED;

	now_secs = get_now_secs();
	if(now_secs != ac_zc_freq_start)
	{
		ac_zc_freq_value = ac_zc_freq_counter;
		ac_zc_freq_counter = 0;
		ac_zc_freq_start = now_secs;
	}
	++ac_zc_freq_counter;

	return IRQ_HANDLED;
}

int __init ac_zc_init(void)
{
	int status;
	printk(KERN_INFO "AC zc v0.1 initializing.\n");

	ac_zc_gpio_previous_value = 0;
	ac_zc_freq_start = get_now_secs();

	status = class_register(&ac_zc_class);
	if(status < 0)
		goto fail_safe;

	status = -EINVAL;
	if(!gpio_is_valid(ac_zc_gpio))
		goto fail_after_class;

	status = gpio_request(ac_zc_gpio, "ac_zc_gpio");
	if(status < 0)
		goto fail_after_class;

	status = gpio_direction_input(ac_zc_gpio);
	if(status < 0)
		goto fail_after_gpio;

	ac_zc_irq = status = gpio_to_irq(ac_zc_gpio);
	if(status < 0)
		goto fail_after_gpio;

	status = request_irq(ac_zc_irq, ac_zc_irq_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_SHARED | IRQF_NO_THREAD, "ac_zc_gpio_irq", &ac_zc_class);
	if(status < 0)
		goto fail_after_gpio;

	printk(KERN_INFO "zc GPIO : %d, IRQ : %d\n", ac_zc_gpio, ac_zc_irq);
	printk(KERN_INFO "AC zc initialized.\n");
	return 0;

fail_after_gpio:
	gpio_free(ac_zc_gpio);
fail_after_class:
	class_unregister(&ac_zc_class);
fail_safe:
	return status;
}

void __exit ac_zc_exit(void)
{
	free_irq(ac_zc_irq, &ac_zc_class);
	gpio_free(ac_zc_gpio);

	class_unregister(&ac_zc_class);
	printk(KERN_INFO "AC zc disabled.\n");
}

/* Copyright (C) 2014 Vincent TRUMPFF
 *
 * May be copied or modified under the terms of the GNU General Public
 * License. See linux/COPYING for more information.
 *
 * Generic software-only driver for AC dimmer
 * (based on zero crossing detection and triac fire angle)
 * via high resolution timers and GPIO lib interface.
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

#include "ac_zc.h"

static int ac_zc_id = -1;

static struct hrtimer hr_timer;

/* dimmer_desc
 *
 * This structure maintains the information regarding a
 * single AC dimmer triac command signal:
 * value : 0 - 100
 */
struct dimmer_desc
{
	int value;
	int gpio_value;
	u64 next_tick;     // timer tick at which next toggling should happen
	unsigned long flags;   // only FLAG_ACDIMMER is used, for synchronizing inside module
#define FLAG_ACDIMMER 1
};

/* dimmer_table
 *
 * The table will hold a description for any GPIO pin available
 * on the system. It's wasteful to preallocate the entire table,
 * but avoiding race conditions is so much easier this way ;-)
*/
static struct dimmer_desc dimmer_table[ARCH_NR_GPIOS];

/* lock protects against dimmer_unexport() being called while
 * sysfs files are active.
 */
static DEFINE_MUTEX(sysfs_lock);

static int dimmer_export(unsigned int gpio);
static int dimmer_unexport(unsigned int gpio);

static void zc_handler(int status, void *data);
static enum hrtimer_restart ac_dimmer_hrtimer_callback(struct hrtimer *timer);
static int mod_init(void);
static void mod_exit(void);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vincent TRUMPFF");
MODULE_DESCRIPTION("Driver for AC dimmer");

module_init(mod_init);
module_exit(mod_exit);

/* Show attribute values for dimmers */
static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct dimmer_desc *desc = dev_get_drvdata(dev);
	ssize_t status;
	mutex_lock(&sysfs_lock);
	if(!test_bit(FLAG_ACDIMMER, &desc->flags))
	{
		status = -EIO;
	}
	else
	{
		status = sysfs_emit(buf, "%d\n", desc->value);
	}
	mutex_unlock(&sysfs_lock);
	return status;
}

/* Store attribute values for dimmers */
static ssize_t value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct dimmer_desc *desc = dev_get_drvdata(dev);
	ssize_t status;
	mutex_lock(&sysfs_lock);
	if(!test_bit(FLAG_ACDIMMER, &desc->flags)){
		status = -EIO;
	}
	else
	{
		unsigned long value;
		status = kstrtoul(buf, 0, &value);
		if(status == 0)
		{
			if(strcmp(attr->attr.name, "value") == 0)
			{
				if(value < 0)
					value = 0;
				if(value > 100)
					value = 100;
				desc->value = value;
			}
		}
	}
	mutex_unlock(&sysfs_lock);
	return status ? : size;
}

static DEVICE_ATTR_RW(value);

static struct attribute *dimmer_attrs[] = {
	&dev_attr_value.attr,
	NULL,
};

static const struct attribute_group dimmer_group = {
	.attrs = dimmer_attrs,
};

static const struct attribute_group *dimmer_groups[] = {
	&dimmer_group,
	NULL
};

/* Export a GPIO pin to sysfs, and claim it for dimmer usage.
 * See the equivalent function in drivers/gpio/gpiolib.c
 */
static ssize_t export_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len)
{
	long gpio;
	int status;

	status = kstrtol(buf, 0, &gpio);
	if(status < 0)
		goto done;

	status = gpio_request(gpio, "ac_dimmer");
	if(status < 0)
		goto done;

	status = gpio_direction_output(gpio,0);
	if(status < 0)
		goto done;

	status = dimmer_export(gpio);
	if(status < 0)
		goto done;

	set_bit(FLAG_ACDIMMER, &dimmer_table[gpio].flags);

done:
	if(status)
	{
		gpio_free(gpio);
		pr_debug("%s: status %d\n", __func__, status);
	}
	return status ? : len;
}

static CLASS_ATTR_WO(export);

/* Unexport a dimmer GPIO pin from sysfs, and unreclaim it.
 * See the equivalent function in drivers/gpio/gpiolib.c
 */
static ssize_t unexport_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len)
{
	long gpio;
	int  status;

	status = kstrtol(buf, 0, &gpio);
	if(status < 0)
		goto done;

	status = -EINVAL;
	if(!gpio_is_valid(gpio))
		goto done;

	if(test_and_clear_bit(FLAG_ACDIMMER, &dimmer_table[gpio].flags))
	{
		status = dimmer_unexport(gpio);
		if(status == 0)
			gpio_free(gpio);
	}
done:
	if(status)
		pr_debug("%s: status %d\n", __func__, status);
	return status ? : len;
}

static CLASS_ATTR_WO(unexport);

static struct attribute *ac_dimmer_class_attrs[] =
{
	&class_attr_export.attr,
	&class_attr_unexport.attr,
	NULL,
};

ATTRIBUTE_GROUPS(ac_dimmer_class);

static struct class ac_dimmer_class =
{
	.name =         "ac_dimmer",
	.owner =        THIS_MODULE,
	.class_groups = ac_dimmer_class_groups,
};


/* Setup the sysfs directory for a claimed dimmer device */
int dimmer_export(unsigned int gpio)
{
	struct dimmer_desc *desc;
	struct device   *dev;
	int             status;

	mutex_lock(&sysfs_lock);

	desc = &dimmer_table[gpio];
	desc->value = 0;
	desc->gpio_value = 0;
	dev = device_create_with_groups(&ac_dimmer_class, NULL, MKDEV(0, 0), desc, dimmer_groups, "dimmer%d", gpio);
	if(dev)
	{
		printk(KERN_INFO "Registered device dimmer%d\n", gpio);
	}
	else
	{
		status = -ENODEV;
	}

	mutex_unlock(&sysfs_lock);

	if(status)
		pr_debug("%s: dimmer%d status %d\n", __func__, gpio, status);
	return status;
}

/* Used by dimmer_unexport below to find the device which should be freed */
static int match_export(struct device *dev, const void *data)
{
	return dev_get_drvdata(dev) == data;
}

/* Free a claimed dimmer device and unregister the sysfs directory */
int dimmer_unexport(unsigned int gpio)
{
	struct dimmer_desc *desc;
	struct device   *dev;
	int             status;

	mutex_lock(&sysfs_lock);

	desc = &dimmer_table[gpio];
	dev  = class_find_device(&ac_dimmer_class, NULL, desc, match_export);
	if(dev)
	{
		put_device(dev);
		device_unregister(dev);
		printk(KERN_INFO "Unregistered device dimmer%d\n", gpio);
		status = 0;
	}
	else
	{
		status = -ENODEV;
	}

	mutex_unlock(&sysfs_lock);

	if(status)
		pr_debug("%s: dimmer%d status %d\n", __func__, gpio, status);
	return status;
}

/* The timer callback is called only when needed (which is to
 * say, at the earliest dimmer signal toggling time) in order to
 * maintain the pressure on system latency as low as possible
 */
enum hrtimer_restart ac_dimmer_hrtimer_callback(struct hrtimer *timer)
{
	unsigned int gpio;
	struct dimmer_desc *desc;
	u64 now = ktime_get_ns();
	u64 next_tick = 0;

	for(gpio=0; gpio<ARCH_NR_GPIOS; gpio++)
	{
		desc = &dimmer_table[gpio];
		if(!test_bit(FLAG_ACDIMMER, &desc->flags))
			continue;

		if(desc->next_tick == 0)
			continue;

		// trigger
		if(desc->next_tick <= now)
		{
			if(desc->gpio_value == 0)
			{
				gpio_set_value(gpio, 1);
				desc->gpio_value = 1;
				desc->next_tick += 300000;
			}
			else
			{
				gpio_set_value(gpio, 0);
				desc->gpio_value = 0;
				// remove trigger
				desc->next_tick = 0;
				continue;
			}
		}

		// timer setup
		if((next_tick == 0) || (desc->next_tick < next_tick))
			next_tick = desc->next_tick;
	}

	if(next_tick > 0)
		hrtimer_start(&hr_timer, next_tick, HRTIMER_MODE_ABS);

	return HRTIMER_NORESTART;
}

void zc_handler(int status, void *data)
{
	unsigned int gpio;
	struct dimmer_desc *desc;
	int period_cent = 0;
	int freq = ac_zc_freq();
	u64 now = ktime_get_ns();
	u64 next_tick = 0;

	if(freq > 0)
		period_cent = (NSEC_PER_SEC / 100) / freq;

	// timer management
	for(gpio=0; gpio<ARCH_NR_GPIOS; gpio++)
	{
		desc = &dimmer_table[gpio];
		if(!test_bit(FLAG_ACDIMMER, &desc->flags))
			continue;

		// reset timer
		desc->next_tick = 0;

		// full time on
		if(desc->value == 100)
		{
			gpio_set_value(gpio, 1);
			continue;
		}

		// period start low
		gpio_set_value(gpio, 0);
		desc->gpio_value = 0;

		// full time off or no period
		if(desc->value == 0 || period_cent == 0)
			continue;

		// timer setup
		// max 90 else it overlaps (timer delay ?)
		desc->next_tick = now + (min(90, (100 - desc->value)) * period_cent);
		if((next_tick == 0) || (desc->next_tick < next_tick))
			next_tick = desc->next_tick;
	}

	if(next_tick > 0)
		hrtimer_start(&hr_timer, next_tick, HRTIMER_MODE_ABS);
}

int __init mod_init(void)
{
	int status;
	printk(KERN_INFO "AC dimmer v0.1 initializing.\n");

	hrtimer_init(&hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hr_timer.function = &ac_dimmer_hrtimer_callback;

	status = class_register(&ac_dimmer_class);
	if(status < 0)
		goto fail_no_class;

	status = ac_zc_register(AC_ZC_STATUS_ENTER, zc_handler, NULL);
	if(status < 0)
		goto fail_zc_register;

	ac_zc_id = status;

	printk(KERN_INFO "AC dimmer initialized.\n");
	return 0;

fail_zc_register:
	class_unregister(&ac_dimmer_class);
fail_no_class:
	return status;
}

void __exit mod_exit(void)
{
	unsigned int gpio;
	int status;

	ac_zc_unregister(ac_zc_id);

	hrtimer_cancel(&hr_timer);

	for(gpio=0; gpio<ARCH_NR_GPIOS; gpio++)
	{
		struct dimmer_desc *desc;
		desc = &dimmer_table[gpio];
		if(test_bit(FLAG_ACDIMMER, &desc->flags))
		{
			gpio_set_value(gpio, 0);
			status = dimmer_unexport(gpio);
			if(status == 0)
				gpio_free(gpio);
		}
	}

	class_unregister(&ac_dimmer_class);
	printk(KERN_INFO "AC dimmer disabled.\n");
}

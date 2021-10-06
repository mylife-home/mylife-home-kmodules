#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/gpio.h>

#include "management.h"
#include "hw.h"
#include "mbox.h"

struct item_desc item_table[ARCH_NR_GPIOS];

static DEFINE_MUTEX(sysfs_lock);

static int item_export(unsigned int gpio);
static int item_unexport(unsigned int gpio);
static int item_export_locked(unsigned int gpio);
static int item_unexport_locked(unsigned int gpio);

static int mod_init(void);
static void mod_exit(void);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vincent TRUMPFF");
MODULE_DESCRIPTION("Driver for DMA PWM");

static char *hw_delay_type = "pcm";
module_param(hw_delay_type, charp, 0444);
MODULE_PARM_DESC(hw_delay_type, "Type of HW device used for delays (supported : pcm,pwm)");

module_init(mod_init);
module_exit(mod_exit);

static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf) {
  ssize_t status;
  const struct item_desc *desc;

  mutex_lock(&sysfs_lock);

  desc = dev_get_drvdata(dev);
  if(!test_bit(FLAG_PWM, &desc->flags)) {
    status = -EIO;
  } else {
    status = sysfs_emit(buf, "%d\n", desc->value);
  }

  mutex_unlock(&sysfs_lock);
  return status;
}

static ssize_t value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
  ssize_t status;
  struct item_desc *desc;

  mutex_lock(&sysfs_lock);

  desc = dev_get_drvdata(dev);
  if(!test_bit(FLAG_PWM, &desc->flags)) {
    status = -EIO;
  } else {

    unsigned long value;
    status = kstrtoul(buf, 0, &value);
    if(status == 0)
		{
      if(value < 0) { value = 0; }
      if(value > 100) { value = 100; }
      desc->value = value;
    }
  }

  mutex_unlock(&sysfs_lock);

  if(status >= 0) {
    hw_update(0);
  }

  return status ? : size;
}

static DEVICE_ATTR_RW(value);

static struct attribute *pwm_attrs[] = {
	&dev_attr_value.attr,
	NULL,
};

static const struct attribute_group pwm_group = {
	.attrs = pwm_attrs,
};

static const struct attribute_group *pwm_groups[] = {
	&pwm_group,
	NULL
};

static ssize_t export_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len) {
  long gpio;
  int  status;

  if((status = kstrtol(buf, 0, &gpio)) < 0) {
    return status;
  }

  if((status = gpio_request(gpio, "dma_pwm")) < 0) {
    return status;
  }

  if((status = gpio_direction_output(gpio, 0)) < 0) {
    gpio_free(gpio);
    return status;
  }

  if((status = item_export(gpio)) < 0) {
    gpio_free(gpio);
    return status;
  }

  set_bit(FLAG_PWM, &item_table[gpio].flags);

  hw_update(0);

  return len;
}

static CLASS_ATTR_WO(export);

static ssize_t unexport_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len) {
  long gpio;
  int  status;

  if((status = kstrtol(buf, 0, &gpio)) < 0) {
    return status;
  }

  if(!gpio_is_valid(gpio)) {
    return -EINVAL;
  }

  if(!test_and_clear_bit(FLAG_PWM, &item_table[gpio].flags)) {
    return -EINVAL;
  }

  if((status = item_unexport(gpio)) < 0) {
    return status;
  }

  hw_update(1);

  gpio_set_value(gpio, 0);
  gpio_free(gpio);
  return len;
}

static CLASS_ATTR_WO(unexport);

static ssize_t delay_type_show(struct class *class, struct class_attribute *attr, char *buf) {
  switch(delay_type) {
  case DELAY_PCM:
    return strscpy(buf, "pcm\n", PAGE_SIZE);

  case DELAY_PWM:
    return strscpy(buf, "pwm\n", PAGE_SIZE);
  }

  return -EFAULT;
}

static CLASS_ATTR_RO(delay_type);

static ssize_t debug_dump_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len) {

  // 'echo registers > debug_dump' add \n at the end
  if(strstr(buf, "registers")) {
    hw_dump_registers();
    return len;
  }

  if(strstr(buf, "dmacb")) {
    hw_dump_dmacb();
    return len;
  }

  if(strstr(buf, "samples")) {
    hw_dump_samples();
    return len;
  }

  return -EINVAL;
}

static CLASS_ATTR_WO(debug_dump);

static struct attribute *dma_pwm_class_attrs[] =
{
	&class_attr_export.attr,
	&class_attr_unexport.attr,
	&class_attr_delay_type.attr,
	&class_attr_debug_dump.attr,
	NULL,
};

ATTRIBUTE_GROUPS(dma_pwm_class);

static struct class dma_pwm_class =
{
	.name =         "dma_pwm",
	.owner =        THIS_MODULE,
	.class_groups = dma_pwm_class_groups,
};

int item_export(unsigned int gpio) {
  int status;

  mutex_lock(&sysfs_lock);
  status = item_export_locked(gpio);
  mutex_unlock(&sysfs_lock);
  return status;
}

int item_export_locked(unsigned int gpio) {
  struct item_desc *desc;
  struct device   *dev;

  desc = &item_table[gpio];
  desc->value = 0;
  desc->dev = dev = device_create_with_groups(&dma_pwm_class, NULL, MKDEV(0, 0), desc, pwm_groups, "pwm%d", gpio);
  if(!dev) {
    return -ENODEV;
  }

  printk(KERN_INFO "Registered device pwm%d\n", gpio);
  return 0;
}

int item_unexport(unsigned int gpio) {
  int status;

  mutex_lock(&sysfs_lock);
  status = item_unexport_locked(gpio);
  mutex_unlock(&sysfs_lock);
  return status;
}

int item_unexport_locked(unsigned int gpio) {
  struct item_desc *desc;
  struct device   *dev;

  desc = &item_table[gpio];
  dev  = desc->dev;
  if(!dev) {
    return -ENODEV;
  }

  device_unregister(dev);
  desc->dev = NULL;

  printk(KERN_INFO "Unregistered device pwm%d\n", gpio);
  return 0;
}

int __init mod_init(void) {
  int status;

  if(strcmp(hw_delay_type, "pcm") == 0) {
    delay_type = DELAY_PCM;
  } else if(strcmp(hw_delay_type, "pwm") == 0) {
    delay_type = DELAY_PWM;
  } else {
    return -EINVAL;
  }

  if((status = mbox_init()) < 0) {
    return status;
  }

  if((status = class_register(&dma_pwm_class)) < 0) {
    return status;
  }

  if((status = hw_init()) < 0) {
    class_unregister(&dma_pwm_class);
    return status;
  }

  printk(KERN_INFO "DMA PWM v1.0 loaded.\n");
  return 0;
}

void __exit mod_exit(void) {
  unsigned int gpio;

  for(gpio=0; gpio<ARCH_NR_GPIOS; ++gpio) {
    struct item_desc *desc = &item_table[gpio];
    if(!test_bit(FLAG_PWM, &desc->flags)) {
      continue;
    }

    desc->value = 0;
  }

  hw_update(1);

  for(gpio=0; gpio<ARCH_NR_GPIOS; ++gpio) {
    struct item_desc *desc = &item_table[gpio];
    if(!test_bit(FLAG_PWM, &desc->flags)) {
      continue;
    }

    item_unexport(gpio);

    gpio_set_value(gpio, 0);
    gpio_free(gpio);
  }

  hw_exit();

  class_unregister(&dma_pwm_class);
  printk(KERN_INFO "DMA PWM v1.0 unloaded.\n");
}

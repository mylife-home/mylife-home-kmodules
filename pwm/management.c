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
static ssize_t attr_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t attr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t attr_show_locked(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t attr_store_locked(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t export_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len);
static ssize_t unexport_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len);
static ssize_t delay_type_show(struct class *class, struct class_attribute *attr, char *buf);
static ssize_t debug_dump_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len);

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

static DEVICE_ATTR(value, 0644, attr_show, attr_store);

static const struct attribute *dev_attrs[] = {
  &dev_attr_value.attr,
  NULL,
};

static const struct attribute_group dev_item = {
  .attrs = (struct attribute **) dev_attrs,
};

static struct class_attribute dev_class_attrs[] = {
  __ATTR_WO(export),
  __ATTR_WO(unexport),
  __ATTR_RO(delay_type),
  __ATTR_WO(debug_dump),
  __ATTR_NULL,
};

static struct class dev_class = {
  .name =        "dma_pwm",
  .owner =       THIS_MODULE,
  .class_attrs = dev_class_attrs,
};

ssize_t attr_show(struct device *dev, struct device_attribute *attr, char *buf) {
  ssize_t status;

  mutex_lock(&sysfs_lock);
  status = attr_show_locked(dev, attr, buf);
  mutex_unlock(&sysfs_lock);
  return status;
}

ssize_t attr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
  ssize_t status;

  mutex_lock(&sysfs_lock);
  status = attr_store_locked(dev, attr, buf, size);
  mutex_unlock(&sysfs_lock);

  if(status >= 0) {
    hw_update(0);
  }

  return status;
}

ssize_t attr_show_locked(struct device *dev, struct device_attribute *attr, char *buf) {
  const struct item_desc *desc = dev_get_drvdata(dev);

  if(!test_bit(FLAG_PWM, &desc->flags)) {
    return -EIO;
  }

  if(strcmp(attr->attr.name, "value") == 0) {
    return snprintf(buf, PAGE_SIZE, "%d\n", desc->value);
  }

  return -EIO;
}

ssize_t attr_store_locked(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
  struct item_desc *desc = dev_get_drvdata(dev);

  if(!test_bit(FLAG_PWM, &desc->flags)) {
    return -EIO;
  }

  if(strcmp(attr->attr.name, "value") == 0) {

    unsigned long value;
    ssize_t status = kstrtoul(buf, 0, &value);
    if(status) {
      return status;
    }

    if(value < 0) { value = 0; }
    if(value > 100) { value = 100; }
    desc->value = value;
    return size;
  }

  return -EIO;
}

ssize_t export_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len) {
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

ssize_t unexport_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len) {
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

ssize_t delay_type_show(struct class *class, struct class_attribute *attr, char *buf) {
  switch(delay_type) {
  case DELAY_PCM:
    return strscpy(buf, "pcm\n", PAGE_SIZE);

  case DELAY_PWM:
    return strscpy(buf, "pwm\n", PAGE_SIZE);
  }

  return -EFAULT;
}

ssize_t debug_dump_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len) {

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

int item_export(unsigned int gpio) {
  int status;

  mutex_lock(&sysfs_lock);
  status = item_export_locked(gpio);
  mutex_unlock(&sysfs_lock);
  return status;
}

int item_export_locked(unsigned int gpio) {
  int status;
  struct item_desc *desc;
  struct device   *dev;

  desc = &item_table[gpio];
  desc->value = 0;
  desc->dev = dev = device_create(&dev_class, NULL, MKDEV(0, 0), desc, "pwm%d", gpio);
  if(!dev) {
    return -ENODEV;
  }

  if((status = sysfs_create_group(&dev->kobj, &dev_item)) < 0) {
    device_unregister(dev);
    return status;
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

  if((status = class_register(&dev_class)) < 0) {
    return status;
  }

  if((status = hw_init()) < 0) {
    class_unregister(&dev_class);
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

  class_unregister(&dev_class);
  printk(KERN_INFO "DMA PWM v1.0 unloaded.\n");
}

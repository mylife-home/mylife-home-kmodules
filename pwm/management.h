#ifndef __MYLIFE_DMA_PWM_MANAGEMENT_H__
#define __MYLIFE_DMA_PWM_MANAGEMENT_H__

struct item_desc {
  struct device *dev;  // corresponding sysfs device
  int value;           // 0 - 100
  unsigned long flags; // only FLAG_PWM is used, for synchronizing inside module
#define FLAG_PWM 1
};

extern struct item_desc item_table[ARCH_NR_GPIOS];

#endif // __MYLIFE_DMA_PWM_MANAGEMENT_H__

#ifndef __MYLIFE_DMA_PWM_HW_H__
#define __MYLIFE_DMA_PWM_HW_H__

enum delay_type {
  DELAY_PCM,
  DELAY_PWM
};

extern enum delay_type delay_type;

extern int hw_init(void); // delay_type MUST be set before
extern void hw_exit(void);
extern void hw_update(int wait);
extern void hw_dump_registers(void);
extern void hw_dump_dmacb(void);
extern void hw_dump_samples(void);

#endif // __MYLIFE_DMA_PWM_HW_H__

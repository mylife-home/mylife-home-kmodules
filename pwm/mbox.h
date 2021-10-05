#ifndef __MYLIFE_DMA_PWM_MBOX_H__
#define __MYLIFE_DMA_PWM_MBOX_H__

// from https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
#define MBOX_MEM_FLAG_DISCARDABLE (1 << 0)    // can be resized to 0 at any time. Use for cached data
#define MBOX_MEM_FLAG_NORMAL (0 << 2)         // normal allocating alias. Don't use from ARM
#define MBOX_MEM_FLAG_DIRECT (1 << 2)         // 0xC alias uncached
#define MBOX_MEM_FLAG_COHERENT (2 << 2)       // 0x8 alias. Non-allocating in L2 but coherent
#define MBOX_MEM_FLAG_ZERO (1 << 4)           // initialise buffer to all zeros
#define MBOX_MEM_FLAG_NO_INIT (1 << 5)        // don't initialise (default is initialise to all ones
#define MBOX_MEM_FLAG_HINT_PERMALOCK (1 << 6) // Likely to be locked for long periods of time.
#define MBOX_MEM_FLAG_L1_NONALLOCATING (MBOX_MEM_FLAG_DIRECT | MBOX_MEM_FLAG_COHERENT) // Allocating in L2

extern int mbox_init(void);

extern int mbox_mem_alloc(uint32_t size, uint32_t align, uint32_t flags, uint32_t *handle);
extern int mbox_mem_free(uint32_t handle);
extern int mbox_mem_lock(uint32_t handle, uint32_t *phys_addr);
extern int mbox_mem_unlock(uint32_t handle);

#endif // __MYLIFE_DMA_PWM_MBOX_H__

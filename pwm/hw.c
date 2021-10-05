// https://github.com/sarfata/pi-blaster/blob/master/pi-blaster.c
// https://github.com/richardghirst/PiBits/blob/master/ServoBlaster/kernel/servoblaster.c
// http://www.valvers.com/wp-content/uploads/2013/01/arm-c-virtual-addresses.jpg
// https://www.kernel.org/doc/gorman/html/understand/understand009.html
// https://stackoverflow.com/questions/38760947/ioremapped-address-in-kernel
// https://github.com/Wallacoloo/Raspberry-Pi-DMA-Example/blob/master/dma-gpio.c

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "management.h"
#include "hw.h"
#include "mbox.h"

// TODO: use device tree : xxd /proc/device-tree/soc/ranges (2nd word)
#if   defined (MYLIFE_ARCH_RPI1)
#define IO_PHYS_BASE 0x20000000
#elif defined (MYLIFE_ARCH_RPI2)
#define IO_PHYS_BASE 0x3f000000
#else
#error "Unknown arch"
#endif

#define IO_BUS_BASE   0x7e000000

#define CYCLE_TIME_US 10000
#define SAMPLE_US     10
#define NUM_SAMPLES   (CYCLE_TIME_US / SAMPLE_US)
#define NUM_CBS       (NUM_SAMPLES * 2)

enum delay_type delay_type;

struct dma_cb {
  uint32_t info;
  uint32_t src;
  uint32_t dst;
  uint32_t length;
  uint32_t stride;
  uint32_t next;
  uint32_t pad[2];
};

struct ctl {
  uint32_t sample[NUM_SAMPLES];
  struct dma_cb cb[NUM_CBS];
};

#define NUM_PAGES ((sizeof(struct ctl) + PAGE_SIZE - 1) >> PAGE_SHIFT)

#define PWM_OFFSET         0x0020C000
#define PWM_LEN            0x28
#define PWM_PHYS_BASE      (IO_PHYS_BASE + PWM_OFFSET)
#define PWM_BUS_BASE       (IO_BUS_BASE  + PWM_OFFSET)

#define PWM_CTL            0x00
#define PWM_STA            0x04
#define PWM_DMAC           0x08
#define PWM_RNG1           0x10
#define PWM_FIFO           0x18

#define PWM_CTL_PWEN1        (1<<0)
#define PWM_CTL_MODE1        (1<<1)
#define PWM_CTL_REPEATEMPTY1 (1<<2)
#define PWM_CTL_USEF1        (1<<5)
#define PWM_CTL_CLRF         (1<<6)
#define PWM_DMAC_ENAB        (1<<31)
#define PWM_DMAC_THRSHLD     ((15<<8) | (15<<0))
#define PWM_STA_BUSERR       (1<<8)
#define PWM_STA_GAPERRS      (0xf << 4)
#define PWM_STA_FIFOREADERR  (1<<3)
#define PWM_STA_FIFOWRITEERR (1<<2)
#define PWM_STA_ERRS         (PWM_STA_BUSERR | PWM_STA_GAPERRS | PWM_STA_FIFOREADERR | PWM_STA_FIFOWRITEERR)

#define PCM_OFFSET         0x00203000
#define PCM_LEN            0x24
#define PCM_PHYS_BASE      (IO_PHYS_BASE + PCM_OFFSET)
#define PCM_BUS_BASE       (IO_BUS_BASE  + PCM_OFFSET)

#define PCM_CS_A           0x00
#define PCM_FIFO_A         0x04
#define PCM_MODE_A         0x08
#define PCM_RXC_A          0x0c
#define PCM_TXC_A          0x10
#define PCM_DREQ_A         0x14
#define PCM_INTEN_A        0x18
#define PCM_INT_STC_A      0x1c
#define PCM_GRAY           0x20

#define CLK_OFFSET         0x00101000
#define CLK_LEN            0xA8
#define CLK_PHYS_BASE      (IO_PHYS_BASE + CLK_OFFSET)
#define CLK_BUS_BASE       (IO_BUS_BASE  + CLK_OFFSET)

#define PCMCLK_CNTL        38
#define PCMCLK_DIV         39
#define PWMCLK_CNTL        40
#define PWMCLK_DIV         41

#define GPIO_OFFSET        0x00200000
#define GPIO_LEN           0x100
#define GPIO_BUS_BASE      (IO_BUS_BASE + GPIO_OFFSET)
#define GPCLR0             0x28
#define GPSET0             0x1c

#define DMA_PHYS_BASE      (IO_PHYS_BASE + 0x00007000)
#define DMA_BUS_BASE       (IO_BUS_BASE + 0x00007000)
#define DMA_CHAN_NUM       14    // the DMA Channel we are using, NOTE: DMA Ch 0 seems to be used by X... better not use it ;)
#define DMA_CHAN_SIZE      0x100 // size of register space for a single DMA channel
#define DMA_CHAN_MAX       14    // number of DMA Channels we have... actually, there are 15... but channel fifteen is mapped at a different DMA_PHYS_BASE, so we leave that one alone
#define DMA_LEN            0x1000
#define DMA_CHAN_OFFSET    (DMA_CHAN_NUM * DMA_CHAN_SIZE)

#define DMA_NO_WIDE_BURSTS  (1<<26)
#define DMA_WAIT_RESP       (1<<3)
#define DMA_D_DREQ          (1<<6)
#define DMA_PER_MAP(x)      ((x)<<16)
#define DMA_END             (1<<1)
#define DMA_RESET           (1<<31)
#define DMA_INT             (1<<2)

#define DMA_CS              0x00
#define DMA_CONBLK_AD       0x04
#define DMA_DEBUG           0x20

// 0xfe0 INT_STATUS Interrupt status of each DMA channel 32
// 0xff0 ENABLE Global enable bits for each DMA channel 32

static void *ctl_addr;
static uint32_t ctl_mbox_handle;
static uint32_t ctl_bus_addr;
static uint32_t ctl_phys_addr;
static void *dma_reg;
static void *clk_reg;
static void *pwm_reg;
static void *pcm_reg;

static void memory_cleanup(void);
static uint32_t virt_to_bus(const void *addr);
static uint32_t bus_to_phys(uint32_t bus_addr);
static uint32_t read_reg(volatile void *reg_base_addr, uint32_t reg_offset);
static void write_reg(volatile void *reg_base_addr, uint32_t reg_offset, uint32_t value);
static void write_reg_and_wait(volatile void *reg_base_addr, uint32_t reg_offset, uint32_t value, unsigned long usecs);
static void or_reg(volatile void *reg_base_addr, uint32_t reg_offset, uint32_t value);
static void or_reg_and_wait(volatile void *reg_base_addr, uint32_t reg_offset, uint32_t value, unsigned long usecs);
static void init_ctrl_data(void);
static void init_hardware(void);
static uint32_t create_set_mask(void);
static uint32_t create_clear_mask(unsigned int sample);

void memory_cleanup(void) {

  if(ctl_addr) {
    memunmap(ctl_addr);
    ctl_addr = NULL;
  }

  if(ctl_bus_addr) {
    mbox_mem_unlock(ctl_mbox_handle);
    ctl_bus_addr = 0;
  }

  if(ctl_mbox_handle) {
    mbox_mem_free(ctl_mbox_handle);
    ctl_mbox_handle = 0;
  }

  if(dma_reg) {
    memunmap(dma_reg);
    dma_reg = NULL;
  }

  if(pwm_reg) {
    memunmap(pwm_reg);
    pwm_reg = NULL;
  }

  if(pcm_reg) {
    memunmap(pcm_reg);
    pcm_reg = NULL;
  }

  if(clk_reg) {
    memunmap(clk_reg);
    clk_reg = NULL;
  }
}

inline uint32_t virt_to_bus(const void *addr) {
  uint32_t offset = (uint8_t *)addr - (uint8_t *)ctl_addr;
  return ctl_bus_addr + offset;
}

inline uint32_t bus_to_phys(uint32_t bus_addr) {
  return bus_addr & ~0xC0000000;
}

inline uint32_t read_reg(volatile void *reg_base_addr, uint32_t reg_offset) {
  volatile char *addr = reg_base_addr;
  addr += reg_offset;
  return * ((volatile uint32_t *)addr);
}

inline void write_reg(volatile void *reg_base_addr, uint32_t reg_offset, uint32_t value) {
  volatile char *addr = reg_base_addr;
  addr += reg_offset;
  * ((volatile uint32_t *)addr) = value;
}

inline void write_reg_and_wait(volatile void *reg_base_addr, uint32_t reg_offset, uint32_t value, unsigned long usecs) {
  write_reg(reg_base_addr, reg_offset, value);
  udelay(usecs);
}

inline void or_reg(volatile void *reg_base_addr, uint32_t reg_offset, uint32_t value) {
  volatile char *addr = reg_base_addr;
  addr += reg_offset;
  * ((volatile uint32_t *)addr) |= value;
}

inline void or_reg_and_wait(volatile void *reg_base_addr, uint32_t reg_offset, uint32_t value, unsigned long usecs) {
  or_reg(reg_base_addr, reg_offset, value);
  udelay(usecs);
}

int hw_init() {
  int status;

  ctl_addr = NULL;
  ctl_mbox_handle = 0;
  ctl_bus_addr = 0;
  dma_reg = NULL;
  pwm_reg = NULL;
  pcm_reg = NULL;
  clk_reg = NULL;

  printk(KERN_INFO "DMA Channel:   %5d\n", DMA_CHAN_NUM);
  printk(KERN_INFO "PWM frequency: %5d Hz\n", 1000000 / CYCLE_TIME_US);

  if((status = mbox_mem_alloc(NUM_PAGES * PAGE_SIZE, PAGE_SIZE, MBOX_MEM_FLAG_L1_NONALLOCATING | MBOX_MEM_FLAG_ZERO, &ctl_mbox_handle)) < 0) {
    memory_cleanup();
    return status;
  }

  if((status = mbox_mem_lock(ctl_mbox_handle, &ctl_bus_addr)) < 0) {
    memory_cleanup();
    return status;
  }

  ctl_phys_addr = bus_to_phys(ctl_bus_addr);

#define CHECK_MEM(x) if(!(x)) { memory_cleanup(); return -ENOMEM; }

  CHECK_MEM(dma_reg = memremap(DMA_PHYS_BASE, DMA_LEN, MEMREMAP_WT));
  CHECK_MEM(pwm_reg = memremap(PWM_PHYS_BASE, PWM_LEN, MEMREMAP_WT));
  CHECK_MEM(clk_reg = memremap(CLK_PHYS_BASE, CLK_LEN, MEMREMAP_WT));
  CHECK_MEM(ctl_addr = memremap(ctl_phys_addr, NUM_PAGES * PAGE_SIZE, MEMREMAP_WB));

  switch(delay_type) {
  case DELAY_PCM:
    CHECK_MEM(pcm_reg = memremap(PCM_PHYS_BASE, PCM_LEN, MEMREMAP_WT));
    break;

  case DELAY_PWM:
    CHECK_MEM(pwm_reg = memremap(PWM_PHYS_BASE, PWM_LEN, MEMREMAP_WT));
    break;
  }

#undef CHECK_MEM

  init_ctrl_data();
  init_hardware();
  hw_update(0);

  return 0;
}

void hw_exit(void) {

  write_reg_and_wait(dma_reg, DMA_CHAN_OFFSET + DMA_CS, DMA_RESET, 10);

  switch(delay_type) {
  case DELAY_PCM:
    write_reg_and_wait(pcm_reg, PCM_CS_A, 0, 100); // Disable PCM block
    break;

  case DELAY_PWM:
    write_reg_and_wait(pwm_reg, PWM_CTL, 0, 10);
    break;
  }

  memory_cleanup();
}

void hw_update(int wait) {

  struct ctl *ctl = ctl_addr;
  int sample;

  // First we turn on the channels that need to be on
  //   Take the first DMA Packet and set it's target to start pulse
  ctl->cb[0].dst = GPIO_BUS_BASE + GPSET0;
  ctl->sample[0] = create_set_mask();

  // Now we go through all the samples and turn the pins off when needed
  for (sample = 1; sample < NUM_SAMPLES; ++sample) {
    ctl->cb[sample*2].dst = GPIO_BUS_BASE + GPCLR0;
    ctl->sample[sample] = create_clear_mask(sample);
  }

  if(wait) {
    mdelay(CYCLE_TIME_US / 1000);
  }
}

void init_ctrl_data(void) {

  struct ctl *ctl = ctl_addr;
  struct dma_cb *cbp = ctl->cb;
  int sample;

  memset(ctl->sample, 0, sizeof(ctl->sample));

  /* Initialize all the DMA commands. They come in pairs.
   *  - 1st command copies a value from the sample memory to a destination
   *    address whichis gpclr0 register
   *  - 2nd command waits for a trigger from an external source (PWM)
   */
  for (sample = 0; sample < NUM_SAMPLES; ++sample) {

    // First DMA command
    cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
    cbp->src = virt_to_bus(ctl->sample + sample);
    cbp->dst = GPIO_BUS_BASE + GPCLR0;
    cbp->length = sizeof(uint32_t);
    cbp->stride = 0;
    cbp->next = virt_to_bus(cbp + 1);
    ++cbp;

    // Second DMA command
    switch(delay_type) {
    case DELAY_PCM:
      cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2);
      cbp->dst = PCM_BUS_BASE + PCM_FIFO_A;
      break;

    case DELAY_PWM:
      cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5);
      cbp->dst = PWM_BUS_BASE + PWM_FIFO;
      break;
    }

    cbp->src = virt_to_bus(ctl); // Any data will do
    cbp->dst = PWM_BUS_BASE + PWM_FIFO;
    cbp->length = sizeof(uint32_t);
    cbp->stride = 0;
    cbp->next = virt_to_bus(cbp + 1);
    ++cbp;
  }

  // point to the first
  --cbp;
  cbp->next = virt_to_bus(ctl->cb);
}

void init_hardware(void) {

  struct ctl *ctl = ctl_addr;

  switch(delay_type) {
  case DELAY_PCM:
    // Initialize PCM
    write_reg_and_wait(pcm_reg, PCM_CS_A, 1, 100); // Disable Rx+Tx, Enable PCM block

    write_reg_and_wait(clk_reg, PCMCLK_CNTL, 0x5A000006, 100); // Source=PLLD (500MHz)
    write_reg_and_wait(clk_reg, PCMCLK_DIV, 0x5A000000 | (500<<12), 100); // Set pcm div to 500, giving 1MHz
    write_reg_and_wait(clk_reg, PCMCLK_CNTL, 0x5A000016, 100); // Source=PLLD and enable

    write_reg_and_wait(pcm_reg, PCM_TXC_A, 0<<31 | 1<<30 | 0<<20 | 0<<16, 100); // 1 channel, 8 bits
    write_reg_and_wait(pcm_reg, PCM_MODE_A, (SAMPLE_US - 1) << 10, 100);
    or_reg_and_wait(pcm_reg, PCM_CS_A, 1<<4 | 1<<3, 100); // Clear FIFOs
    write_reg_and_wait(pcm_reg, PCM_DREQ_A, 64<<24 | 64<<8, 100); // DMA Req when one slot is free?
    or_reg_and_wait(pcm_reg, PCM_CS_A, 1<<9, 100); // Enable DMA
    break;

  case DELAY_PWM:
    // Initialize PWM
    write_reg_and_wait(pwm_reg, PWM_CTL, 0, 10);

    write_reg_and_wait(clk_reg, PWMCLK_CNTL, 0x5A000006, 100); // Source=PLLD (500MHz)
    write_reg_and_wait(clk_reg, PWMCLK_DIV, 0x5A000000 | (500<<12), 100); // set pwm div to 500, giving 1MHz
    write_reg_and_wait(clk_reg, PWMCLK_CNTL, 0x5A000016, 100); // Source=PLLD and enable

    write_reg(pwm_reg, PWM_DMAC, 0); //disable DMA
    or_reg_and_wait(pwm_reg,  PWM_CTL, PWM_CTL_CLRF, 100); //clear pwm
    write_reg_and_wait(pwm_reg, PWM_STA, PWM_STA_ERRS, 100); //clear PWM errors
    write_reg_and_wait(pwm_reg, PWM_DMAC, PWM_DMAC_ENAB | PWM_DMAC_THRSHLD, 10);
    write_reg_and_wait(pwm_reg, PWM_RNG1, SAMPLE_US, 10);
    write_reg_and_wait(pwm_reg, PWM_CTL, PWM_CTL_USEF1 | PWM_CTL_PWEN1 | PWM_CTL_REPEATEMPTY1, 10);
    break;
  }

  // Initialize the DMA
  write_reg_and_wait(dma_reg, DMA_CHAN_OFFSET + DMA_CS, DMA_RESET, 10);
  write_reg(dma_reg, DMA_CHAN_OFFSET + DMA_CS, DMA_INT | DMA_END);
  write_reg(dma_reg, DMA_CHAN_OFFSET + DMA_CONBLK_AD, virt_to_bus(ctl->cb));
  write_reg_and_wait(dma_reg, DMA_CHAN_OFFSET + DMA_DEBUG, 7, 10); // clear debug error flags
  write_reg(dma_reg, DMA_CHAN_OFFSET + DMA_CS, 0x10880001); // go, mid priority, wait for outstanding writes

  if(delay_type == DELAY_PCM) {
    or_reg(pcm_reg, PCM_CS_A, 1<<2); // Enable Tx
  }
}

inline uint32_t create_set_mask(void) {
  uint32_t mask = 0;
  unsigned int gpio;

  for(gpio=0; gpio<ARCH_NR_GPIOS; ++gpio) {
    struct item_desc *desc = &item_table[gpio];
    if(!test_bit(FLAG_PWM, &desc->flags)) {
      continue;
    }

    // value = 0 -> no pulse
    if(desc->value == 0) {
      continue;
    }

    mask |= 1 << gpio;
  }

  return mask;
}

inline uint32_t create_clear_mask(unsigned int sample) {
  uint32_t mask = 0;
  unsigned int gpio;

  for(gpio=0; gpio<ARCH_NR_GPIOS; ++gpio) {
    struct item_desc *desc = &item_table[gpio];
    if(!test_bit(FLAG_PWM, &desc->flags)) {
      continue;
    }

    // max value = 100
    if(desc->value * NUM_SAMPLES > sample * 100) {
      continue;
    }

    mask |= 1 << gpio;
  }

  return mask;
}

void hw_dump_registers(void) {
  struct ctl *ctl = ctl_addr;
  int i;

  printk(KERN_INFO "ctl: %p\n", ctl);
  printk(KERN_INFO "ctl_bus_addr: %p\n", (void *)ctl_bus_addr);

  switch(delay_type) {
  case DELAY_PCM:
    printk(KERN_INFO "pcm_reg: %p\n", pcm_reg);
    for (i=0; i<PCM_LEN/4; ++i) {
      printk(KERN_INFO "%04x: 0x%08x\n", i * 4, ((uint32_t*)pcm_reg)[i]);
    }
    break;

  case DELAY_PWM:
    printk(KERN_INFO "pwm_reg: %p\n", pwm_reg);
    for (i=0; i<PWM_LEN/4; ++i) {
      printk(KERN_INFO "%04x: 0x%08x\n", i * 4, ((uint32_t*)pwm_reg)[i]);
    }
    break;
  }

  printk(KERN_INFO "clk_reg: %p\n", clk_reg);
  for (i=0; i<CLK_LEN/4; ++i) {
    printk(KERN_INFO "%04x: 0x%08x\n", i * 4, ((uint32_t*)clk_reg)[i]);
  }

  printk(KERN_INFO "dma_reg: %p\n", (char*)dma_reg + DMA_CHAN_OFFSET);
  for (i=0; i<DMA_CHAN_SIZE/4; ++i) {
    printk(KERN_INFO "%04x: 0x%08x\n", i * 4, ((uint32_t*)((char*)dma_reg + DMA_CHAN_OFFSET))[i]);
  }
}

void hw_dump_dmacb(void) {
  struct ctl *ctl = ctl_addr;
  int i;
  struct dma_cb *cbp = ctl->cb;

  for (i = 0; i < NUM_SAMPLES * 2; i++) {
    printk(KERN_INFO "DMA Control Block: #%d @0x%px, bus=0x%08x\n", i, cbp, virt_to_bus(cbp));
    printk(KERN_INFO "info:   0x%08x\n", cbp->info);
    printk(KERN_INFO "src:    0x%08x\n", cbp->src);
    printk(KERN_INFO "dst:    0x%08x\n", cbp->dst);
    printk(KERN_INFO "length: 0x%08x\n", cbp->length);
    printk(KERN_INFO "stride: 0x%08x\n", cbp->stride);
    printk(KERN_INFO "next:   0x%08x\n", cbp->next);
    cbp++; // next control block
  }
}

void hw_dump_samples(void) {
  struct ctl *ctl = ctl_addr;
  int i;

  for (i = 0; i < NUM_SAMPLES; ++i) {
    printk(KERN_INFO "#%d 0x%08x\n", i, ctl->sample[i]);
  }
}
// https://github.com/raspberrypi/linux/blob/rpi-4.9.y/drivers/char/broadcom/vcio.c
// https://github.com/sarfata/pi-blaster/blob/master/mailbox.c

#include <linux/kernel.h>
#include <soc/bcm2835/raspberrypi-firmware.h>

#include "mbox.h"

static struct rpi_firmware *fw;

struct mbox_msg_header {
  uint32_t tag;
  uint32_t buffer_size;
  uint32_t data_size;
};

struct mbox_msg_alloc {
  struct mbox_msg_header header;
  union {
    uint32_t ret;

    struct {
      uint32_t size;
      uint32_t align;
      uint32_t flags;
    } payload;
  };
};

struct mbox_msg_handle {
  struct mbox_msg_header header;
  union {
    uint32_t ret;
    struct {
      uint32_t handle;
    } payload;
  };
};

#define MBOX_MSG_INIT(msg, tagid) \
  { \
    msg.header.tag = tagid; \
    msg.header.buffer_size = msg.header.data_size = sizeof(msg.payload); \
  }

static int mbox_property(void *data, size_t tag_size);

int mbox_init(void) {
  struct device_node *np;

  np = of_find_compatible_node(NULL, NULL, "raspberrypi,bcm2835-firmware");

  if(!of_device_is_available(np)) {
    return -ENODEV;
  }

  if (!(fw = rpi_firmware_get(np))) {
    return -ENODEV;
  }

  return 0;
}

inline int mbox_property(void *data, size_t tag_size) {
  return rpi_firmware_property_list(fw, data,tag_size);
}

int mbox_mem_alloc(uint32_t size, uint32_t align, uint32_t flags, uint32_t *handle) {
  struct mbox_msg_alloc msg;
  int ret;

  MBOX_MSG_INIT(msg, RPI_FIRMWARE_ALLOCATE_MEMORY);
  msg.payload.size = size;
  msg.payload.align = align;
  msg.payload.flags = flags;

  ret = mbox_property(&msg, sizeof msg);
  if(ret == 0) {
    *handle = msg.ret;
  }

  return ret;
}

int mbox_mem_free(uint32_t handle) {
  struct mbox_msg_handle msg;

  MBOX_MSG_INIT(msg, RPI_FIRMWARE_RELEASE_MEMORY);
  msg.payload.handle = handle;

  return mbox_property(&msg, sizeof msg);
}

int mbox_mem_lock(uint32_t handle, uint32_t *phys_addr) {
  struct mbox_msg_handle msg;
  int ret;

  MBOX_MSG_INIT(msg, RPI_FIRMWARE_LOCK_MEMORY);
  msg.payload.handle = handle;

  ret = mbox_property(&msg, sizeof msg);
  if(ret == 0) {
    *phys_addr = msg.ret;
  }

  return ret;
}

int mbox_mem_unlock(uint32_t handle) {
  struct mbox_msg_handle msg;

  MBOX_MSG_INIT(msg, RPI_FIRMWARE_UNLOCK_MEMORY);
  msg.payload.handle = handle;

  return mbox_property(&msg, sizeof msg);
}


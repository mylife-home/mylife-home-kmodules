# mylife-home-kmodules
MyLife Home Linux kernel modules

## AC

Provide `ac_zc`, `ac_dimmer` and `ac_button`

Modules built from rpi-alpine-build

Development:
```shell
make -C dev-tools/Makefile.ac <rule>
```

## PWM

Provide `dma_pwm`

Modules built from rpi-alpine-build

Development:
```shell
make -C dev-tools/Makefile.pwm <rule>
```

# Documentatione

## sysfs
 - https://github.com/torvalds/linux/blob/master/Documentation/core-api/kobject.rst
 - https://github.com/torvalds/linux/blob/master/samples/kobject/kobject-example.c
 - https://github.com/torvalds/linux/blob/master/drivers/gpio/gpiolib-sysfs.c
 - https://sysprog21.github.io/lkmpg/#sysfs-interacting-with-your-module
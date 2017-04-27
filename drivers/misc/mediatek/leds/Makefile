#include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/misc/mediatek/leds/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include

obj-$(CONFIG_MTK_LEDS) += leds_drv.o
obj-$(CONFIG_NUBIA_PMIC_LEDS) += nubia_led_drv.o

obj-$(CONFIG_ZTEMT_BREATH_LED_AW2013)           += aw2013.o

obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/


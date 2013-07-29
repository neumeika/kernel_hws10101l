# Makefile for the light drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_S10_FEATURE_PROXIMITY_APDS990X)  += apds990x.o

obj-$(CONFIG_LIGHT_SENSOR_PO188) += po188.o

ifeq ($(CONFIG_PROXIMITY_LIGHT_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif

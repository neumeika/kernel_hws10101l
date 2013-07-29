#
# Makefile for the accelerometer drivers.
#
#sensor unification for differ platform
# Each configuration option enables a list of files.

obj-$(CONFIG_S10_FEATURE_SENSORS_ACCELEROMETER_ADI_ADXL346)	+= adxl34x.o
obj-$(CONFIG_S10_FEATURE_SENSORS_ACCELEROMETER_MMA8452)	+= gs_mma8452.o
obj-$(CONFIG_S10_FEATURE_SENSORS_ACCELEROMETER_ST_LIS3XH)	+= lis3dh_acc.o

obj-$(CONFIG_S10_FEATURE_SENSORS_ACCELEROMETER_KIONIX_KXTIK)  += kxtik.o

ifeq ($(CONFIG_GSENSOR_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif

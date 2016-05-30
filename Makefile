#
# Makefile for Invensense inv-mpu-iio device.
#

obj-$(CONFIG_INV_MPU9150_IIO) += inv-mpu-iio.o

inv-mpu-iio-objs := inv_mpu_core.o
inv-mpu-iio-objs += inv_mpu_ring.o
inv-mpu-iio-objs += inv_mpu_trigger.o
inv-mpu-iio-objs += inv_mpu_misc.o
inv-mpu-iio-objs += inv_slave_compass.o


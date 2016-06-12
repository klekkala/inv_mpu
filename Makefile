#
# Makefile for Invensense MPU6050 device.
#

obj-$(CONFIG_INV_MPU6050_IIO) += inv-mpu6050.o
inv-mpu6050-objs := inv_mpu_core.o inv_mpu_ring.o inv_mpu_trigger.o dmpDefaultMPU6050.o

obj-$(CONFIG_INV_MPU9250_IIO) += inv-mpu9250.o
inv-mpu9250-objs := inv_mpu_core.o inv_mpu_ring.o inv_mpu_trigger.o dmpDefaultMPU6050.o inv_slave_compass.o

obj-$(CONFIG_INV_MPU6050_I2C) += inv-mpu6050-i2c.o
inv-mpu6050-i2c-objs := inv_mpu_i2c.o inv_mpu_acpi.o

obj-$(CONFIG_INV_MPU6050_SPI) += inv-mpu6050-spi.o
inv-mpu6050-spi-objs := inv_mpu_spi.o

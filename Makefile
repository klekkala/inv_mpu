#
# Makefile for Invensense inv-mpu-iio device.
# Makefile modified for vanilla 4.4x kernel
#

obj-m += inv-mpu-iio.o inv_mpu_core.o inv_mpu_ring.o inv_mpu_trigger.o inv_mpu_misc.o inv_mpu3050_iio.o dmpDefaultMPU6050.o inv_slave_compass.o inv_slave_pressure.o

all: make -C /lib/modules/4.1.17-ti-rt-r47/build/ M=. modules
clean: make -C /lib/modules/4.1.17-ti-rt-r47/build/ M=. clean

#
# Makefile for Microsemi DSP
#

obj-y += ssl.o hbi.o hbi_tw.o hbi_k.o

ccflags-y += -DVPROC_MAX_NUM_DEVS=1 -DNUM_MAX_LOCKS=100 -DHBI_ENABLE_PROCFS=1 \
	-DCHIP=38051 -DHBI_BUFFER_SIZE=1024 -DHOST_ENDIAN_LITTLE=1 -DVPROC_DEV_ENDIAN_LITTLE=0 \
	-DDEBUG_LEVEL=8 -DDEBUG -DI2C=1 -DSPI=2

# TW = 1
ccflags-y += -DTW=1 -DTARGET=TW

# I2C = 1
#ccflags-y += -DHBI=I2C
#obj-y += hal_i2c.o

# SPI = 2
ccflags-y += -DHBI=SPI
obj-y += hal_spi.o
#!/usr/bin/env python

import smbus
import time


bus =smbus.SMBus(1)	# 0 = /dev/i2c-0 (port I2C0) 1 = /dev/i2c-1 (port I2C1)

DEVICE_ADDRESS = 0x07
DEVICE_REG_MODE1 = 0x00
DEVICE_REG_LEDOUT0 = 0x1d
value = 0x21

bus.write_byte_data(DEVICE_ADDRESS, 0, value)
data = bus.read_i2c_block_data(DEVICE_ADDRESS, 0, 8) #read 8 bytes at device_address with offset 0

print data
time.sleep(1)

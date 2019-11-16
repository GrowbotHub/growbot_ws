#!/usr/bin/env python

from gpiozero import Device, DigitalOutputDevice
import json
import os
import time
import smbus
import rospy
from growbothub_tlc.srv import DeviceReadWrite, DeviceSummary


if 'DRY_RUN' in os.environ:
    from gpiozero.pins.mock import MockFactory
    Device.pin_factory = MockFactory()
else:
    from gpiozero.pins.native import NativeFactory
    Device.pin_factory = NativeFactory()


class DeviceManager():
    '''
    Manages list of devices
    '''
    _devices = {}

    @staticmethod
    def add(idx, device):
        DeviceManager._devices[idx] = device

    @staticmethod
    def get(idx):
        return DeviceManager._devices[idx]

    @staticmethod
    def summary():
        devices_info = {}
        for dev_id, dev in DeviceManager._devices.items():
            devices_info[dev_id] = dev.summary()
        return devices_info


class DeviceInterface():
    '''
    Provides an abstraction for easy access from HTTP API:
    - dictionary commands &
    - device information.
    '''
    def write(self, json):
        raise NotImplementedError()

    def read(self, command):
        raise NotImplementedError()

    def info(self):
        raise NotImplementedError()


class RelayDevice(DeviceInterface):
    def __init__(self, pin=5):
        self.dev = None
        self.dev = DigitalOutputDevice(pin)

    def write(self, command):
        self.dev.value = int(command['value'])

    def read(self, command):
        return { 'value': self.dev.value }

    def summary(self):
        return { 'type': 'relay', 'pin': str(self.dev.pin) }


class CameraDevice(DeviceInterface):
    def summary(self):
        return { 'type': 'camera' }


class TH02Device(DeviceInterface):
    def __init__(self, bus=None):
        self._bus = bus
        if self._bus is None:
            try:
                self._bus = smbus.SMBus(1)
            except Exception as e:
                print(e)

    def get_temperature_humidity(self):
        self._bus.write_byte_data(0x40, 0x03, 0x11)
        time.sleep(0.5)
        data = self._bus.read_i2c_block_data(0x40, 0x00, 3)
        temp = ((data[1] * 256 + (data[2] & 0xFC))/ 4.0) / 32.0 - 50.0

        self._bus.write_byte_data(0x40, 0x03, 0x01)
        time.sleep(0.5)
        data = self._bus.read_i2c_block_data(0x40, 0x00, 3)
        humidity = ((data[1] * 256 + (data[2] & 0xF0)) / 16.0) / 16.0 - 24.0
        humidity = humidity - (((humidity * humidity) * (-0.00393)) + (humidity * 0.4008) - 4.7844)
        humidity = humidity + (temp - 30) * (humidity * 0.00237 + 0.1973)

        return temp, humidity

    def read(self, command):
        temp, humidity = self.get_temperature_humidity()
        return { 'temperature': temp, 'humidity': humidity }

    def summary(self):
        return { 'type': 'TH02' }


def device_read_cb(args):
    device = DeviceManager.get(args.device_id)
    res = device.read('')
    return str(json.dumps(res))


def device_write_cb(args):
    device = DeviceManager.get(args.device_id)
    device.write(json.loads(args.command))
    return ''


def device_summary_cb(args):
    return str(json.dumps(DeviceManager.summary()))


if __name__ == "__main__":
    DeviceManager.add('lights', RelayDevice(5))
    DeviceManager.add('temp', TH02Device())
    DeviceManager.add('camera', CameraDevice())
    print(DeviceManager.summary())

    rospy.init_node('devices')
    rospy.Service('device_read', DeviceReadWrite, device_read_cb)
    rospy.Service('device_write', DeviceReadWrite, device_write_cb)
    rospy.Service('device_summary', DeviceSummary, device_summary_cb)
    rospy.spin()

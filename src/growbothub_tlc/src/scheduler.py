#!/usr/bin/env python

import schedule 
import time 
import json
import rospy
import os
import base64
import sys
from datetime import datetime
from growbothub_tlc.srv import DeviceReadWrite


if sys.version_info[0] < 3 or sys.version_info[1] < 4:
    # python version < 3.3
    import time
    def timestamp(date):
        return time.mktime(date.timetuple())
else:
    def timestamp(date):
        return date.timestamp()


device_read = None
device_write = None
DIST_DATA = os.environ['DIST_DATA']


def turn_lights_on(): 
    device_write('lights', '{ "value": "1" }')
  

def turn_lights_off(): 
    device_write('lights', '{ "value": "0" }')
  

def take_picture():
    folder = os.path.join(DIST_DATA, 'images')
    if not os.path.exists(folder):
		os.makedirs(folder)

    timestamp_str = datetime.now().strftime('%y-%m-%d_%H-%M-%S')
    img_name = 'image_{}.jpg'.format(timestamp_str)
    path = os.path.join(folder, img_name)

    readings = json.loads(device_read('camera', '').readings)
    image = base64.b64decode(readings['base64'].split(',')[1])
    with open(path, 'wb') as f:
        f.write(image)


def meassure_temperature():
    epoch = timestamp(datetime.now())
    readings = device_read('temp', '').readings
    params = json.loads(readings)
    with open(os.path.join(DIST_DATA, 'temp.csv'), 'a') as f:
        f.write('{},{},{}\n'.format(epoch, params['temperature'], params['humidity']))


def system_verification():
    time.sleep(1)
    print('Testing devices...')
    meassure_temperature()

    if json.loads(device_read('lights', '').readings)['value'] == '1':
        turn_lights_off()
        turn_lights_on()
        take_picture()
    else:
        turn_lights_on()
        time.sleep(1)
        take_picture()
        turn_lights_off()
    take_picture()
    print('Devices tested')


if __name__ == "__main__":
    rospy.init_node('scheduler')
    r = rospy.Rate(1)

    schedule.every(60).minutes.do(take_picture) 
    schedule.every(1).minutes.do(meassure_temperature) 
    schedule.every().day.at('07:00').do(turn_lights_on) 
    schedule.every().day.at('20:00').do(turn_lights_off) 

    device_read = rospy.ServiceProxy('device_read', DeviceReadWrite)
    device_write = rospy.ServiceProxy('device_write', DeviceReadWrite)

    system_verification()

    while not rospy.is_shutdown():
        schedule.run_pending()
        r.sleep()

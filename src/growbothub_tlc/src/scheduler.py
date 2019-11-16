#!/usr/bin/env python

import schedule 
import time 
import json
import rospy
import os
from datetime import datetime
from growbothub_tlc.srv import DeviceReadWrite


device_read = None
device_write = None
DIST_DATA = os.environ['DIST_DATA']


def turn_lights_on(): 
    print('turn_lights_on') 
  

def turn_lights_off(): 
    print('turn_lights_off') 
  

def take_picture(): 
    print('take picture') 


def meassure_temperature():
    timestamp = datetime.now().timestamp()
    params = json.loads(device_read('temp', ''))
    with open(os.path.join(DIST_DATA, 'temp.csv'), 'a') as f:
        f.write('{},{},{}\n'.format(timestamp, params['temperature'], params['humidity']))



if __name__ == "__main__":
    rospy.init_node('scheduler')
    r = rospy.Rate(1)

    schedule.every(60).minutes.do(take_picture) 
    schedule.every(30).minutes.do(meassure_temperature) 
    schedule.every().day.at('07:00').do(turn_lights_on) 
    schedule.every().day.at('20:00').do(turn_lights_off) 

    device_read = rospy.ServiceProxy('device_read', DeviceReadWrite)
    device_write = rospy.ServiceProxy('device_write', DeviceReadWrite)

    while not rospy.is_shutdown():
        schedule.run_pending()
        r.sleep()

#!/usr/bin/env python
#test for git
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

#def meassure_temperature():
#    epoch = timestamp(datetime.now())
#    readings = device_read('temp', '').readings
#    params = json.loads(readings)
#    with open(os.path.join(DIST_DATA, 'temp.csv'), 'a') as f:
#        f.write('{},{},{}\n'.format(epoch, params['temperature'], params['humidity']))

def turn_pumps_off():
    device_write('pumps', json.dumps({'value': 0}))


def turn_pumps_on():
    device_write('pumps', json.dumps({'value': 1}))

def system_verification():
    time.sleep(1)
    print('Testing devices...')
    meassure_temperature()

    turn_lights_on()
    turn_pumps_on()
    time.sleep(3)
    turn_lights_off()
    turn_pumps_off()
    take_picture()
    print('Devices tested')
    now = time.localtime().tm_hour
    if now >= 7 and now <= 20:
        turn_lights_on()

if __name__ == "__main__":
    rospy.init_node('scheduler')
    r = rospy.Rate(1)

    schedule.every(60).minutes.do(take_picture) 
#    schedule.every(1).minutes.do(meassure_temperature) 
    schedule.every().day.at('07:00').do(turn_lights_on) 
    schedule.every().day.at('20:00').do(turn_lights_off) 
    schedule.every().hour.at(':00').do(turn_pumps_on)
    schedule.every().hour.at(':01').do(turn_pumps_off)
    schedule.every().hour.at(':05').do(turn_pumps_on)
    schedule.every().hour.at(':06').do(turn_pumps_off)
    schedule.every().hour.at(':10').do(turn_pumps_on)
    schedule.every().hour.at(':12').do(turn_pumps_off)
    schedule.every().hour.at(':16').do(turn_pumps_on)
    schedule.every().hour.at(':18').do(turn_pumps_off)
    schedule.every().hour.at(':22').do(turn_pumps_on)
    schedule.every().hour.at(':24').do(turn_pumps_off)
    schedule.every().hour.at(':28').do(turn_pumps_on)
    schedule.every().hour.at(':30').do(turn_pumps_off)
    schedule.every().hour.at(':34').do(turn_pumps_on)
    schedule.every().hour.at(':36').do(turn_pumps_off)
    schedule.every().hour.at(':40').do(turn_pumps_on)
    schedule.every().hour.at(':42').do(turn_pumps_off)
    schedule.every().hour.at(':46').do(turn_pumps_on)
    schedule.every().hour.at(':48').do(turn_pumps_off)
    schedule.every().hour.at(':52').do(turn_pumps_on)
    schedule.every().hour.at(':54').do(turn_pumps_off)
    schedule.every().hour.at(':58').do(turn_pumps_on)
    schedule.every().hour.at(':59').do(turn_pumps_off)

    device_read = rospy.ServiceProxy('device_read', DeviceReadWrite)
    device_write = rospy.ServiceProxy('device_write', DeviceReadWrite)

    system_verification()

    while not rospy.is_shutdown():
        schedule.run_pending()
        r.sleep()

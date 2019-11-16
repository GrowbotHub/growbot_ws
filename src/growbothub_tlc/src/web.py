#!/usr/bin/env python

import os
import json
from flask import Flask, request
import rospy
from growbothub_tlc.srv import DeviceReadWrite, DeviceSummary


app = Flask(__name__)
device_read = None
device_write = None
device_summary = None


@app.route('/device/<string:device_id>/read', methods=['GET'])
def http_device_read(device_id):
    res = device_read(device_id, '')
    return json.loads(res.readings)


@app.route('/device/<string:device_id>/write', methods=['GET'])
def http_device_write(device_id):
    device_write(device_id, json.dumps(request.args))
    res = device_read(device_id, '')
    return json.loads(res.readings)


@app.route('/device/summary', methods=['GET'])
def http_device_summary():
    res = device_summary().summary
    return json.loads(res)
    

if __name__ == "__main__":
    rospy.init_node('web')
    device_read = rospy.ServiceProxy('device_read', DeviceReadWrite)
    device_write = rospy.ServiceProxy('device_write', DeviceReadWrite)
    device_summary = rospy.ServiceProxy('device_summary', DeviceSummary)

    port = int(os.environ.get('PORT', 8000))
    app.debug = True
    app.run(host='0.0.0.0', port=port, threaded=True)

#!/usr/bin/env python

import json
import os
import io
import time
import rospy
import subprocess
from std_msgs.msg import String


def classify():
    cur_dir = os.path.dirname(os.path.realpath(__file__))
    classifier_file = os.path.join(cur_dir, 'classifier_cli.py')

    f = open(os.path.join(cur_dir, 'test.jpg'), 'rb')
    res = subprocess.check_output(['python3', classifier_file], stdin=f)

    message = json.loads(res.decode('UTF-8').rstrip())
    return float(message['classification'])


if __name__ == "__main__":
    rospy.init_node('classifier')
    pub = rospy.Publisher('classify', String)
    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        res = classify()
        pub.publish(str(res))
        rate.sleep()

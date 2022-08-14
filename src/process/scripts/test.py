#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from tm_msgs.srv import *
import tf
import geometry_msgs.msg
import numpy as np
from std_msgs.msg import String, Int32MultiArray, Float64MultiArray


def tm_send_script_client(cmd):
    rospy.wait_for_service('tm_driver/send_script')
    try:
        tm_send_script = rospy.ServiceProxy('tm_driver/send_script', SendScript)
        resp1 = tm_send_script("demo", cmd)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def hole_status_callback(data):
    print(data.data)


def get_hole_status_plate():
    print()


if __name__ == "__main__":

    a = ()
    b = []

    b.append()

    c = np.zeros(5)
    d = c.tolist()
    e = np.array(d)
    rospy.init_node('demo_send_script', anonymous=True)

    sub_hole_status = rospy.Subscriber('/hole_status', String, hole_status_callback, queue_size=1)
    # tm_send_script_client('ScriptExit(0)')
    # tm_send_script_client("PTP(\"CPP\",600,0,400,0,180,-90,100,200,0,false)")
    # listener = tf.TransformListener()
    # rate = rospy.Rate(10.0)
    # while not rospy.is_shutdown():
    #     try:
    #         t = listener.getLatestCommonTime("/base", "/tool0")
    #         (trans, rot) = listener.lookupTransform('/base', '/tool0', t)
    #         print(trans, rot)
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    rospy.spin()

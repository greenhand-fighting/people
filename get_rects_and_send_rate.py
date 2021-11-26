#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from beginner_tutorials.msg import Rate
from opencv_apps.msg import RectArrayStamped
from cv_bridge import CvBridge, CvBridgeError

rate_threshold=0.05
no_detect_num=0
is_people=0
total_mianji=1080*1080
pub=rospy.Publisher("rate", Rate, queue_size=10)

def callback(data):
    global no_detect_num, is_people, rate_threshold, total_mianji
    print("no_detect_num: %d" % no_detect_num)
    mianjis=[]
    if len(data.rects):
        for rect in data.rects:       
            mianji=rect.width * rect.height
            mianjis.append(mianji)
            max_mianji=max(mianjis)
            print(max_mianji)
            rate=max_mianji/total_mianji
            print("RATE: %f" % rate)
            if rate>=rate_threshold:
                no_detect_num=0
                is_people=1
                pass
            else:
                no_detect_num+=1
                if no_detect_num>10:
                    is_people=0
                pass
            pub.publish(is_people)
    else:
        print("no people detect!")
        no_detect_num+=1
        if no_detect_num>10:
            is_people=0
            pub.publish(is_people)
        #pub.publish(0.0)

def callback2(data):
    total_mianji=data.height * data.width
    print(total_mianji)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('get_rate', anonymous=True)
    if total_mianji:
        pass
    else :
        rospy.Subscriber("/people_detect/image", Image, callback2)

    rospy.Subscriber("/people_detect/found", RectArrayStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    listener()
    rospy.spin()
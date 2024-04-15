#!/usr/bin/env python3
import rospy
from flexy_hand.msg import EMGdata
import csv
import time
import os

TIME_WINDOW = 20 # seconds

muscle1 = []
muscle2 = []
muscle3 = []
muscle4 = []

def callback(data):
    muscle1.append(data.muscle1)
    muscle2.append(data.muscle2)
    muscle3.append(data.muscle3)
    muscle4.append(data.muscle4)

def save_mvc():
    with open(os.path.join("src/flexy_hand/scripts/data", "mvc.txt"), 'w') as csvfile:
        writer = csv.writer(csvfile)
        data = [max(muscle1), max(muscle2), max(muscle3), max(muscle4)]
        writer.writerow(data)
    
def listener():

    rospy.init_node('data_saver', anonymous=False)
    rospy.Subscriber("emg", EMGdata, callback)

    rospy.loginfo("Recording data...")
    prev_time = time.time()
    while not rospy.is_shutdown():
        if time.time() - prev_time > TIME_WINDOW:
            save_mvc()
            rospy.loginfo("MVC saved!")
            break

    #rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
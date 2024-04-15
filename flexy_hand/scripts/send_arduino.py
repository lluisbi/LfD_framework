#!/usr/bin/env python3
import rospy
from flexy_hand.msg import FingersAngles
import serial

ser = serial.Serial('/dev/ttyACM0', 9600) # Adjust the port and baud rate as needed

def remap_value(val, OldMin, OldMax, NewMin, NewMax):
    return (((val - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard: \n%s", data)
    thumb_ang = round(remap_value(-1*data.thumb, -180, -100, 10, 140))
    index_ang = round(remap_value(-1*data.index, -180, -80,  10, 180))
    long_ang  = round(remap_value(-1*data.long,  -180, -80,  10, 180))
    ring_ang  = round(remap_value(-1*data.ring,  -180, -80,  10, 180))
    small_ang = round(remap_value(-1*data.small, -180, -80,  10, 180))
    m = [thumb_ang, index_ang, small_ang, long_ang, ring_ang]
    m_str = ""
    for i in range(0,5):
        m_str = m_str + "{:03d}".format(m[i])
    m_str = m_str + '\n'
    ser.write(m_str.encode('utf-8'))
    
def listener():
    rospy.init_node('send_arduino', anonymous=False)
    rospy.Subscriber("fingers_angles", FingersAngles, callback)
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
#!/usr/bin/env python3
import rospy
from flexy_hand.msg import FingersAngles
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

def remap_value(val, OldMin, OldMax, NewMin, NewMax):
    return (((val - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard: \n%s", data)
    thumb_ang = round(remap_value(-1*data.thumb, -180, -100, 0, 45))
    index_ang = round(remap_value(-1*data.index, -180, -80,  0, 45))
    long_ang  = round(remap_value(-1*data.long,  -180, -80,  0, 45))
    ring_ang  = round(remap_value(-1*data.ring,  -180, -80,  0, 45))
    small_ang = round(remap_value(-1*data.small, -180, -80,  0, 45))

    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['hand_body__thumb0', 'thumb0__thumb1', 
                'hand_body__index0', 'index0__index1', 'index1__index2',
                'hand_body__long0',  'long0__long1',   'long1__long2',
                'hand_body__ring0',  'ring0__ring1',   'ring1__ring2',
                'hand_body__small0', 'small0__small1', 'small1__small2']
    msg.position = [math.radians(thumb_ang), math.radians(thumb_ang),      # Thumb
                    math.radians(index_ang), math.radians(index_ang), math.radians(index_ang), # Index
                    math.radians(long_ang),  math.radians(long_ang),  math.radians(long_ang), # Long
                    math.radians(ring_ang),  math.radians(ring_ang),  math.radians(ring_ang), # Ring
                    math.radians(small_ang), math.radians(small_ang), math.radians(small_ang)] # Small
    msg.velocity = []
    msg.effort = []

    rospy.loginfo(msg)
    pub.publish(msg)
    
def listener():
    rospy.init_node('send_sim', anonymous=False)
    rospy.Subscriber("fingers_angles", FingersAngles, callback)
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math


def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(30) # 30hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['link_4_to_leg_base','j_leg_01_to_leg_base', 'j_leg_02_to_leg1']
    # hello_str.position = [3, 0.5418]
    hello_str.velocity = []
    hello_str.effort = []
    L1 = 0.02
    L2 = 0.25
    L3 = 0.20
    t = 0
    while not rospy.is_shutdown():
        hello_str.header.stamp = rospy.Time.now()
        x = -1*(0.3+0.1*math.sin(t))
        z = 0.2* math.cos(t)
        #y = 0.1
        #y = 0.2 * math.cos(t)
        # x = 0
        y = 0.00000000000000000001
        # z = 0.45
        C = x*x + y*y -L1*L1
        D = (C + z*z - L2*L2 - L3 * L3)/(2*L2*L3)
        q3 = math.atan2(math.sqrt(1- D*D),D)
        if(C < 0) : C = 0
        q1 = math.atan2(-y,x) - math.atan2(math.sqrt(C),L1)
        q2 = math.atan2(z, math.sqrt(C)) \
            - math.atan2(L3*math.sin(q3),L2+L3*math.cos(q3))



        t = t + 0.15
        hello_str.position = [q1, q2, q3]
        ##hello_str.position = [q1, q2]

        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

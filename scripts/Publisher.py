#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt8MultiArray


if __name__ == '__main__':
    try :
        rospy.init_node('publisher',anonymous=True)
        pub = rospy.Publisher('gain', UInt8MultiArray, queue_size=10)
        rate = rospy.Rate(10)
        speed = 0
        up_gain = 1
        down_gain = 1
        while not rospy.is_shutdown():
            msg = UInt8MultiArray()
            print('input from 0-255 and type Q to leave')
            speed = input('speed = ')
            up_gain = input('accel = ')
            down_gain = input('deccel = ')
            if(speed == 'Q' or up_gain == 'Q' or down_gain == 'Q'):
                break
            msg.data = [int(speed), int(up_gain), int(down_gain)]
            pub.publish(msg)
            rate.sleep()

    except rospy.ROSInterruptException : 
        pass
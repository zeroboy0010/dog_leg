#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import UInt8MultiArray
import odrive
from odrive.enums import *
import math
import time


class odriverun :
    def __init__(self):
        self.odrive = False
        if self.odrive == True : 
            self.mydrive = odrive.find_any()
            self.Calibration()

        self.sub = rospy.Subscriber('gain',UInt8MultiArray, queue_size=10, callback = self.gain_callback)
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.timer_pub = rospy.Timer(rospy.Duration(1/100),callback = self.publisher)

        self.I1 = 26  ## cm noted
        self.I2 = 32
        self.t = math.pi
        self.a = 0

        ## msg


    def Calibration(self):
        # mode
        self.mydrive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.mydrive.axis0.controller.config.input_mode = 5
        self.mydrive.axis0.trap_traj.config.accel_limit = 10
        self.mydrive.axis0.trap_traj.config.decel_limit = 10
        self.mydrive.axis0.trap_traj.config.vel_limit = 35

        self.mydrive.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.mydrive.axis1.controller.config.input_mode = 5
        self.mydrive.axis1.trap_traj.config.accel_limit = 10
        self.mydrive.axis1.trap_traj.config.decel_limit = 10
        self.mydrive.axis1.trap_traj.config.vel_limit = 35
        ############################

        if(self.mydrive.axis0.current_state == AXIS_STATE_IDLE or self.mydrive.axis1.current_state == AXIS_STATE_IDLE):
        # if(1):
            print('rematch the leg')
            time.sleep(5)
            self.mydrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.mydrive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            while self.mydrive.axis0.current_state != AXIS_STATE_IDLE or self.mydrive.axis1.current_state != AXIS_STATE_IDLE:
                print("starting calibration...")
                ## print(str(self.mydrive.axis0.encoder.pos_estimate) + str(self.mydrive.axis1.encoder.pos_estimate))
                time.sleep(0.1)
            self.mydrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.mydrive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        print('ok wait')
        print(str(self.mydrive.axis0.encoder.pos_estimate) + str(self.mydrive.axis1.encoder.pos_estimate))
        time.sleep(2)
        print('starting ....................')
            # set new accel and decel
        gain_up = 5
        gain_down = 5
        self.mydrive.axis0.trap_traj.config.accel_limit = gain_up
        self.mydrive.axis0.trap_traj.config.decel_limit = gain_down
        self.mydrive.axis1.trap_traj.config.accel_limit = gain_up
        self.mydrive.axis1.trap_traj.config.decel_limit = gain_down
        time.sleep(1)

    def gain_callback(self, msg):
        self.a = float(msg.data[0])/1000
        self.upgain = msg.data[1]
        self.downgain = msg.data[2]
        if self.odrive == True :
            self.mydrive.axis0.trap_traj.config.accel_limit = self.upgain
            self.mydrive.axis0.trap_traj.config.decel_limit = self.downgain
            self.mydrive.axis1.trap_traj.config.accel_limit = self.upgain
            self.mydrive.axis1.trap_traj.config.decel_limit = self.downgain

    def publisher(self, event):
        jointmsg = JointState()
        jointmsg.header = Header()
        jointmsg.name = ['j_leg_01_to_leg_base', 'j_leg_02_to_leg1']
        jointmsg.velocity = []
        jointmsg.effort = []
        jointmsg.header.stamp = rospy.Time.now()
        x = 40 + 10*math.sin(-self.t)
        y = 20 * math.cos(self.t)
        q2 = - math.pi + math.acos((self.I1*self.I1 + self.I2*self.I2 - x*x - y*y)/(2*self.I1*self.I2)) # q2 < 0
        # q2 = math.acos((x*x + y*y - self.I1*self.I1 - self.I2*self.I2)/(2*self.I1*self.I2))  # q2 > 0
        q1 = math.atan(y/x) - math.atan((self.I2* math.sin(q2))/(self.I1 + self.I2 * math.cos(q2)))
        angle1 = q1
        angle2 = (math.pi/2) + q2

        angle1out = angle1 * (14)/(2*math.pi)
        angle2out = angle2 * (14)/(2*math.pi)

        if(self.odrive == True):
            self.mydrive.axis0.controller.input_pos =  -1*angle1out
            self.mydrive.axis1.controller.input_pos = 1*angle2out

            feedback_1 = self.mydrive.axis0.encoder.pos_estimate
            feedback_2 = self.mydrive.axis1.encoder.pos_estimate

            back_angle1 = -1*feedback_1 * (2*math.pi)/14
            back_angle2 = -(math.pi/2) + feedback_2 * (2*math.pi)/14

            rospy.loginfo("v1_output = "+str(float(self.mydrive.axis0.encoder.vel_estimate))+", v2_output = "+str(float(self.mydrive.axis1.encoder.vel_estimate)))
        else :
            back_angle1 = q1
            back_angle2 = q2
            rospy.loginfo("q1 = "+ str(q1) + " q2 = " + str(q2))

        self.t = self.t + self.a

        jointmsg.position = [back_angle1, back_angle2]
        self.pub.publish(jointmsg)


if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_publisher',anonymous=False)
        odriverun()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

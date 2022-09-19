import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math


l1 = 0.25 # cm
l2 = 0.20 # cm
m1 = 1 # kg
m2 = 1 # kg
g = 9.8 # m/s^2

#forward kinematic
def get_xy (q1 , q2):
    return [l1 * math.cos(q1) + l2 * math.cos(q1 + q2),
            l1 * math.sin(q1) + l2 * math.sin(q1 + q2)]

#inverse kinematic
def get_q12 (x, y, i):
    if(i == 1):
        q2 = math.acos((x*x + y*y - l1*l1 - l2*l2)/(2 * l1 * l2))
    elif(i == -1):
        q2 = math.pi - math.acos(-1 * (x*x + y*y - l1*l1 - l2*l2)/(2 * l1 * l2))
    q1 = math.atan(y/x) - math.atan((l2 * math.sin(q2))/(l1 + l2 * math.cos(q2)))
    return [q1,q2]

#inverse dynamic
def get_T12 (q1,q2,dot_q1,dot_q2,ddot_q1,ddot_q2):
    M1_11 = ((1/3)*m1*l1*l1 + m2*l1*l1 + (1/3)*m2*l2*l2 + m2*l1*l2*math.cos(q2))
    M1_12 = (1/3)*m2*l2*l2 +(1/2)*m2*l1*l2*math.cos(q2)
    M1_21 = (1/3)*m2*l2*l2 + (1/2)*m2*l1*l2*math.cos(q2)
    M1_22 = (1/3)*m2*l2*l2
    
    M2_12 = (-1/2)*m2*l1*l2*math.sin(q2)
    M2_21 = (1/2)*m2*l1*l2*math.sin(q2)
    
    M3_11 = -m2*l1*l2*math.sin(q2)
    
    M4_1 = ((-1/2)*m1 + m2)*g*l1*math.cos(q1)+ (-1/2)*m2*g*l2*math.cos(q1+q2)
    M4_2 = (-1/2)*m2*g*l2*math.cos(q1+q2)
    
    T1 = (M1_11 * ddot_q1 + M1_12 * ddot_q2) + (M2_12 * dot_q2 * dot_q2)  + (M3_11*dot_q1*dot_q2) + M4_1
    T2 = (M1_21 * ddot_q1 + M1_22 * ddot_q2) + (M2_21 * dot_q1 * dot_q1)  + M4_2
    return [T1, T2]


def talker():
    Sample_time = 0.01 # second
    dot_q1 = 0
    ddot_q1 = 0
    dot_q2 = 0
    ddot_q2 = 0
    #################################
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(100) # 100hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['j_leg_01_to_leg_base', 'j_leg_02_to_leg1']
    # hello_str.position = [3, 0.5418]
    hello_str.velocity = []
    hello_str.effort = []

    t = 0
    old_q1 = 0
    old_q2 = 0
    old_dot_q1 = 0
    old_dot_q2 = 0
    while not rospy.is_shutdown():
        y = 0.30 + 0.06*math.sin(-0.08*t)
        x = 0.10 * math.cos(0.08*t)
        # y = 0.30
        # x = 0.00000001+0.05*math.sin(0.1*t)
        [q1,q2] = get_q12(x,y,1)
        ###################################  
        ## find dot
        new_q1 = q1
        new_q2 = q2

        dot_q1 = (new_q1 - old_q1)/Sample_time
        dot_q2 = (new_q2 - old_q2)/Sample_time

        ## find ddot
        new_dot_q1 = dot_q1
        new_dot_q2 = dot_q2

        ddot_q1 = (new_dot_q1 - old_dot_q1)/Sample_time
        ddot_q2 = (new_dot_q2 - old_dot_q2)/Sample_time
    
        ## continue
        old_q1 = new_q1
        old_q2 = new_q2
        old_dot_q1 = new_dot_q1
        old_dot_q2 = new_dot_q2
        #######################################

        #torque of 2 joints
        [T1,T2] = get_T12 (q1,q2,dot_q1,dot_q2,ddot_q1,ddot_q2)

        t+=1
        #####################################
        hello_str.header.stamp = rospy.Time.now()
        x1 = y
        y1 = -x
        [q1,q2] = get_q12(x1,y1,1)
        hello_str.position = [q1, q2]
        pub.publish(hello_str)
        rospy.loginfo("T1 = "+str(float(T1))+", T2 = "+str(float(T2)))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
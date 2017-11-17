#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import JointState


def run_turntable():

    # Node initialization for the turntable player
    rospy.init_node('turntable')
    crank_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    # Publish JointState information at 50Hz
    update_rate = rospy.Rate(50)

    # Initial configuration of JointState messages and initialize any sequential header variables
    j_pos = JointState()
    seq_num = 0

    # Set initialized time for this function
    time_init = rospy.Time.now()

    # Main calculation / command submission loop
    while not rospy.is_shutdown():

        time_cur = rospy.Time.now()
        time_elps = time_cur - time_init

        J1ang, J2ang = IK_2R_2L(time_elps)

        # Increment the sequence ID for Joint data
        seq_num += 1

        j_pos.header.seq = seq_num
        j_pos.header.stamp = time_cur
        j_pos.header.frame_id = 'EE'

        j_pos.name = ['J1','J2']
        j_pos.position = [J1ang,J2ang]

        # Publish updated position data, then sleep until necessary
        crank_pub.publish(j_pos)

        update_rate.sleep()

def IK_2R_2L(time_elps):

    # Robot arm length definition
    L1 = 1
    L2 = 1

    # Define target positions at current simulation time
    x = 0.5 * math.cos(2 * math.pi * time_elps.to_sec() / 5.0) + 1.25
    y = 0.5 * math.sin(2 * math.pi * time_elps.to_sec() / 5.0)

    # Angular parameters for 2R robot
    alpha = math.acos((x**2 + y**2 + L1**2 - L2**2) / (2 * L1 * (x**2 + y**2)**0.5))
    beta = math.acos((L1**2 + L2**2 - x**2 - y**2 ) / (2 * L1 * L2))
    gamma = math.atan2(y,x)

    # Return joint angles from numerical solution to 2-arm IKs
    J1t = gamma - alpha
    J2t = math.pi - beta

    return J1t, J2t


def main():

    try:
        run_turntable()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

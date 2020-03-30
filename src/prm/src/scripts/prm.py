#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import random
import math

pub_joints = rospy.Publisher('/joint_states', JointState, queue_size=10)

def pubJointState(thetas):
 
    # Create JointState object
    j = JointState()
    j.header.stamp = rospy.Time.now()
    j.name = ['joint1', 'joint2', 'joint3']
    j.position = thetas
    j.velocity = []
    j.effort = []
 
 
    # Publish it
    pub_joints.publish(j)

def generate_configs():
    return [random.uniform(-math.pi, math.pi), random.uniform(-math.pi, math.pi), random.uniform(-math.pi, math.pi)]

def main():
    rospy.init_node('prm', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pubJointState(generate_configs())
        rate.sleep()

if __name__ == '__main__':
    main()
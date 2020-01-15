#!/usr/bin/env python
import rospy
from std_msgs.msg import String, UInt32

def callback_a(data):
    print(data)

def callback_b(data):
    print(data)

def main():
    rospy.init_node('node_c', anonymous=True)
    rospy.Subscriber('topic_a', String, callback_a)
    rospy.Subscriber('topic_b', UInt32, callback_b)

    rospy.spin()

if __name__ == '__main__':
    main()
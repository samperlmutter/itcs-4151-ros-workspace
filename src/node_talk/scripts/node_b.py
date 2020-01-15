#!/usr/bin/env python
import rospy
from std_msgs.msg import String, UInt32

def callback(data):
    print(data)

def main():
    rospy.init_node('node_b', anonymous=True)
    rospy.Subscriber('topic_a', String, callback)

    pub = rospy.Publisher('topic_b', UInt32, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(42)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
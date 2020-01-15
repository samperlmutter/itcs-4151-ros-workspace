#!/usr/bin/env python
import rospy
from std_msgs.msg import String, UInt32

def callback(data):
    print(data)

def main():
    rospy.init_node('node_a', anonymous=True)
    pub = rospy.Publisher('topic_a', String, queue_size=10)
    rate = rospy.Rate(10)

    rospy.Subscriber('topic_b', UInt32, callback)

    while not rospy.is_shutdown():
        msg = 'hello from a at {}'.format(rospy.get_time())
        pub.publish(msg)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
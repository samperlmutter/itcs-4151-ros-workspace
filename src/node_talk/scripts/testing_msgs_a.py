#!/usr/bin/env python
import rospy
from node_talk.msg import MyMsg

def main():
    rospy.init_node('testing_msgs_a', anonymous=True)
    pub = rospy.Publisher('mymsg_a', MyMsg, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = MyMsg()
        msg.id = 3
        msg.message = "shalom"
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
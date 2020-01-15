#!/usr/bin/env python
import rospy
from node_talk.msg import MyMsg

def callback(data):
    print("id: {}, message: {}".format(data.id, data.message))

def main():
    rospy.init_node('testing_msgs_b', anonymous=True)
    rospy.Subscriber('mymsg_a', MyMsg, callback)

    rospy.spin()

if __name__ == '__main__':
    main()
#!/usr/bin/env python
import rospy

def main():
    rospy.init_node('my_node', anonymous=True)
    print('Hello ROS!')
    print('-- Sam Perlmutter')

if __name__ == '__main__':
    main()
#!/usr/bin/env python
 
#import statements#
# We're going to need: rospy, tf_conversions, and tf2_ros.
import rospy
import tf_conversions
import tf2_ros
import math
 
# Here we are getting the tf broadcaster and message that will be used.
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
 
#Main function#
def main():
    # Initialize the node.
    rospy.init_node('tf_prep', anonymous=True)
 
    # Make a new broadcaster and transform message.
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
 
    # Set the transform frame ids.
    t.header.frame_id = 'myFrame'
    t.child_frame_id = 'world'
 
    # Set the translation component.
    t.transform.translation.x = 1
    t.transform.translation.y = 1
    t.transform.translation.z = 1
 
    # Set the rotation component
    '''
   ROS uses quaternions to represent rotations, so that part is done for you.
   You only need to set the angle values around each axis in the quaternion_from_euler call
   The units are RADIANS! The order is X, Y, Z. You can import math and use math.pi to help.
   '''
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, math.pi / 4)
    # Then we assign each quaternion parameters to the message.
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
 
    # Wait a second
    rospy.sleep(1)

    # Send the transform message with the broadcaster
    br.sendTransform(t)
    
    # Keeps broadcasting the transform
    rospy.spin()

#This is what will get executed#
if __name__ == '__main__':
    # Run the main function.
    main()
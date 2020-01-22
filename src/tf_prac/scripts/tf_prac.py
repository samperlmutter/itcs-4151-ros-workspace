#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
import math
import numpy as np

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

#mat is a x4 numpy array    
def convertMatToTf(mat):
    result = TransformStamped()
    
    # Translation is set based on position vector in transformation matrix
    result.transform.translation.x = mat[0,3]
    result.transform.translation.y = mat[1,3]
    result.transform.translation.z = mat[2,3]

    ###########################################################
    # Get the rotation values around each axis
    ###########################################################
    
    # If rotation around y is 90 or -90
    if (mat[2,0] >= -1.01 and mat[2,0] <= -0.99) or (mat[2,0] >= 0.99 and mat[2,0] <= 1.01):

        # Set rot_z to anything, usually 0 is selected
        rot_z = 0.0
        if (mat[2,0] >= -1.01 and mat[2,0] <= -0.99):
            rot_y = math.pi / 2.0
            rot_x = rot_z + math.atan2(mat[0,1], mat[0,2])
        else:
            rot_y = -math.pi / 2.0
            rot_x = -rot_z + math.atan2(-mat[0,1], -mat[0,2])

    # Else, rot around y is not 90,-90
    else:
        rot_y = -math.asin(mat[2,0])
        #rot_y_2 = math.pi - rot_y
        
        rot_x = math.atan2(mat[2,1] / math.cos(rot_y), mat[2,2] / math.cos(rot_y))
        #rot_x_2 = math.atan2(mat[2][1] / math.cos(rot_y_2), mat[2][2] / math.cos(rot_y_2))
        
        rot_z = math.atan2( mat[1,0] / math.cos(rot_x), mat[0,0] / math.cos(rot_x))
        #rot_z_2 = math.atan2( mat[1][0] / math.cos(rot_x_2), mat[0][0] / math.cos(rot_x_2))

    # Get a Quaternion based on the euler angle rotations
    q = tf_conversions.transformations.quaternion_from_euler(rot_x, rot_y, rot_z)

    # Set rotation
    result.transform.rotation.x = q[0]
    result.transform.rotation.y = q[1]
    result.transform.rotation.z = q[2]
    result.transform.rotation.w = q[3]

    return result

def translate(x, y, z):
    return np.matrix([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def rotate_x(angle):
    return np.matrix([
        [1, 0, 0, 0],
        [0, math.cos(angle), -math.sin(angle), 0],
        [0, math.sin(angle), math.cos(angle), 0],
        [0, 0, 0, 1]
    ])

def rotate_y(angle):
    return np.matrix([
        [math.cos(angle), -math.sin(angle), 0, 0],
        [0, 1, 0, 0],
        [math.sin(angle), 0, math.cos(angle), 0],
        [0, 0, 0, 1]
    ])

def rotate_z(angle):
    return np.matrix([
        [math.cos(angle), -math.sin(angle), 0, 0],
        [math.sin(angle), math.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def rotate(x, y, z):
    return rotate_z(z) * rotate_y(y) * rotate_x(x)

def transform(x, y, z, rot_x, rot_y, rot_z):
    return rotate(rot_x, rot_y, rot_z) * translate(x, y, z)

def main():
    rospy.init_node('tf_prac', anonymous=True)
 
    br = tf2_ros.TransformBroadcaster()

    my_tf_matrix = np.matrix([
        [math.cos(math.pi / 4), -math.sin(math.pi / 4), 0, 2],
        [math.sin(math.pi / 4), math.cos(math.pi / 4), 0, 3],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    my_tf = convertMatToTf(my_tf_matrix)
    my_tf.header.frame_id = 'world'
    my_tf.child_frame_id = 'myTf'
 
    rospy.sleep(1)

    br.sendTransform(my_tf)

    tran_test = convertMatToTf(translate(1, 2, 1))
    tran_test.header.frame_id = 'world'
    tran_test.child_frame_id = 'tranTest'

    br.sendTransform(tran_test)

    transform_test = convertMatToTf(transform(-1, -2, 1, 0, 0, math.pi / 4))
    transform_test.header.frame_id = 'world'
    transform_test.child_frame_id = 'transformTest'

    br.sendTransform(transform_test)
    
    rospy.spin()

if __name__ == '__main__':
    main()
#!/usr/bin/env python
import sys

import ros_numpy  # apt install ros-noetic-ros-numpy
import rospy
from sensor_msgs.msg import PointCloud2
import pickle
import PyKDL
import tf
import numpy as np

rospy.init_node("aserve", anonymous=True)

def frame_to_tf_matrix(frame):
    matrix = np.zeros((4, 4))
    matrix[0, 0] = frame.M[0, 0]
    matrix[1, 0] = frame.M[1, 0]
    matrix[0, 1] = frame.M[0, 1]
    matrix[1, 1] = frame.M[1, 1]
    matrix[2, 0] = frame.M[2, 0]
    matrix[2, 1] = frame.M[2, 1]
    matrix[2, 2] = frame.M[2, 2]
    matrix[0, 2] = frame.M[0, 2]
    matrix[1, 2] = frame.M[1, 2]
    matrix[0, 3] = frame.p.x()
    matrix[1, 3] = frame.p.y()
    matrix[2, 3] = frame.p.z()
    matrix[3, 3] = 1
    return matrix

listener = tf.TransformListener()
def get_tf(src, dst):
    global listener
    listener.waitForTransform(src, dst, rospy.Time(), rospy.Duration(1))
    pos, rot = listener.lookupTransform(src, dst, rospy.Time(0))
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3]), PyKDL.Vector(pos[0], pos[1], pos[2]))



# /head_kinect/depth/points
# /head_kinect/depth_registered/points

msg = rospy.wait_for_message("/head_kinect/depth/points", PointCloud2, timeout=None)
rgb = ros_numpy.numpify(msg)
#xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=False)

arr = ros_numpy.point_cloud2.split_rgb_field(rgb)
transform1 = frame_to_tf_matrix(get_tf("head_kinect_rgb_optical_frame", "world"))
transform2 = frame_to_tf_matrix(get_tf("world", "head_kinect_rgb_optical_frame"))

print transform2.shape


with open('4_sim.pickle', 'wb') as handle:
    pickle.dump(arr, handle, protocol=pickle.HIGHEST_PROTOCOL)
with open("4_sim_tf1.pickle", 'wb') as handle:
    pickle.dump(transform1, handle, protocol=pickle.HIGHEST_PROTOCOL)
with open("4_sim_tf2.pickle", 'wb') as handle:
    pickle.dump(transform2, handle, protocol=pickle.HIGHEST_PROTOCOL)

print "finished"

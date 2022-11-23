#!/usr/bin/env python
import PyKDL
import tf
import rospy
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseArray, Pose
from tf_maths import PyKDLFrame_to_Pose, tf_matrix_to_PyKDLFrame

def np_to_pykdl_vector(l):
    return PyKDL.Vector(l[0], l[1], l[2])


def publish_detected_object_tf(tf_matrices, names):
    for tf_matrix, name in zip(tf_matrices, names):
        x = tf_matrix[0, 3]
        y = tf_matrix[1, 3]
        z = tf_matrix[2, 3]
        rot = Rotation.from_dcm(tf_matrix[0:3,0:3])
        q = rot.as_quat()
        qx = q[0]
        qy = q[1]
        qz = q[2]
        qw = q[3]
        br = tf.TransformBroadcaster()
        br.sendTransform((x, y, z),
                         (qx, qy, qz, qw),
                         rospy.Time.now(),
                         name,
                         "world")

def publish_pose_arr(frames, pub_name):
    """
    Input - PyKDL frame
    """
    poses = [PyKDLFrame_to_Pose(frame) for frame in frames]
    pub = rospy.Publisher(pub_name, PoseArray, queue_size=10)
    for i in range(50):
        pa = PoseArray()
        pa.poses = poses
        pa.header.frame_id = "world"
        pa.header.stamp = rospy.Time.now()
        pa.header.seq = i
        pub.publish(pa)
        rospy.sleep(0.1)


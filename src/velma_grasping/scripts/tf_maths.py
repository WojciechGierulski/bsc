import numpy as np
import PyKDL
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

def tf_msg_to_matrix(tf):
    M = PyKDL.Rotation.Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
    matrix = np.zeros((4, 4))
    matrix[0, 0] = M[0, 0]
    matrix[1, 0] = M[1, 0]
    matrix[0, 1] = M[0, 1]
    matrix[1, 1] = M[1, 1]
    matrix[2, 0] = M[2, 0]
    matrix[2, 1] = M[2, 1]
    matrix[2, 2] = M[2, 2]
    matrix[0, 2] = M[0, 2]
    matrix[1, 2] = M[1, 2]
    matrix[0, 3] = tf.translation.x
    matrix[1, 3] = tf.translation.y
    matrix[2, 3] = tf.translation.z
    matrix[3, 3] = 1
    return matrix

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

def PyKDLFrame_to_Pose(frame):
    p = Pose()
    p.position.x = frame.p.x()
    p.position.y = frame.p.y()
    p.position.z = frame.p.z()
    qx,qy,qz,qw = frame.M.GetQuaternion()
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


def tf_matrix_to_PyKDLFrame(tf):
    print tf
    rot = Rotation.from_dcm(tf[0:3, 0:3])
    print(rot)
    q = rot.as_quat()
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]

    f = PyKDL.Frame(PyKDL.Rotation.Quaternion(qx,qy,qz,qw), PyKDL.Vector(tf[0,3], tf[1,3], tf[2,3]))
    return f
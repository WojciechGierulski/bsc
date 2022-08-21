import numpy as np
import PyKDL

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
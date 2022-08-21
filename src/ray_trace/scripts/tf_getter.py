import sys

import rospy
import tf
import PyKDL


class TfGetter:
    def __init__(self):
        self.listener = tf.TransformListener()


    def get_tf(self, src, dst):
        self.listener.waitForTransform(src, dst, rospy.Time(), rospy.Duration(1))
        pos, rot = self.listener.lookupTransform(src, dst, rospy.Time(0))
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3]), PyKDL.Vector(pos[0], pos[1], pos[2]))

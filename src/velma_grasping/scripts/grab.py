#!/usr/bin/env python
import sys
import roslib;

roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import PyKDL
import rospkg
from velma_common import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
from rcprg_ros_utils import exitError
from Initializer import Initializer
from sensor_msgs.msg import PointCloud2
from ray_trace.srv import ray_trace
from tf_maths import tf_msg_to_matrix
from std_msgs.msg import Bool

def get_tf_matrix(publish=False):
    rospy.wait_for_service('ray_trace')
    try:
        srv = rospy.ServiceProxy('ray_trace', ray_trace)
        b = Bool()
        b.data = publish
        resp = srv(b)
        if resp.status.data == 1:
            print ("No intersection points!")
            return None
        elif resp.status.data == 0:
            matrix = tf_msg_to_matrix(resp.transform)
            return matrix
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit()


if __name__ == "__main__":
    rospy.init_node('rt_velma', anonymous=False)
    rospack = rospkg.RosPack()
    rt_path = rospack.get_path("ray_trace")

    velma = Initializer.initialize_system()
    #Initializer.move_to_init_pose(velma, rt_path)
    rospy.sleep(0.1)

    tf_matrix = get_tf_matrix(True)
    print tf_matrix



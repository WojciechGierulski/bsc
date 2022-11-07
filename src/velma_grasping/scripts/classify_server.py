#!/usr/bin/env python3

from visual_analysis_functions import process_kinect_data, classify_pc
from velma_grasping.srv import classify, classifyResponse
from objects_databse import DataBase
import rospkg
import rospy
import ros_numpy
from std_msgs.msg import String, Float32
import numpy as np
from geometry_msgs.msg import Pose

db = DataBase()
db.load_db()

def list2Typelist(list, type):
    new_list = []
    for el in list:
        el2 = type()
        type.data = el
        new_list.append(el2)
    return new_list


def Typelist2list(list):
    return [el.data for el in list]


def handle_request(req):
    tf_world_to_cam = np.array(Typelist2list(req.world_to_cam_transform)).reshape((4, 4))
    tf_calib = np.array(Typelist2list(req.calib_transform)).reshape((4, 4))
    pc_msg = req.pc
    data = ros_numpy.point_cloud2.split_rgb_field(ros_numpy.numpify(pc_msg))
    classes = []
    scores = []
    poses = []

    clusters = process_kinect_data(data, tf_world_to_cam, tf_calib)
    for cluster in clusters:
        transform, fitness, name = classify_pc(cluster, db)
        classes.append(name)
        scores.append(fitness)
        pose = Pose()
        pose.position

    return classifyResponse(1.0, "class", None)

if __name__ == "__main__":
    rospy.init_node('classify_server')
    rospack = rospkg.RosPack()
    s = rospy.Service('classify', classify, handle_request)
    print("Classify server ready.")
    rospy.spin()



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
from velma_grasping.msg import FlatTransformation

db = DataBase()
db.load_db()

def TypeListToList(TypeList):
    return [el.data for el in TypeList]

def ListToTypeList(list, type):
    new_list = []
    for el in list:
        el2 = type()
        el2.data = el
        new_list.append(el2)
    return new_list

def create_FlatTransformation(tf):
    ft = FlatTransformation()
    ft.data = tf.flatten().tolist()
    return ft


def handle_request(req):
    tf_world_to_cam = np.array(TypeListToList(req.world_to_cam_transform)).reshape((4, 4))
    tf_calib = np.array(TypeListToList(req.calib_transform)).reshape((4, 4))
    pc_msg = req.pc
    data = ros_numpy.point_cloud2.split_rgb_field(ros_numpy.numpify(pc_msg))
    classes = []
    scores = []
    poses = []

    clusters = process_kinect_data(data, tf_world_to_cam, tf_calib)
    for cluster in clusters:
        transform, fitness, name = classify_pc(cluster, db)
        if name is not None:
            classes.append(name)
            scores.append(fitness)
            poses.append(transform)
    print(poses)
    poses = [create_FlatTransformation(tf) for tf in poses]
    scores = ListToTypeList(scores, Float32)
    classes = ListToTypeList(classes, String)
    return classifyResponse(classes, scores, poses)

if __name__ == "__main__":
    rospy.init_node('classify_server')
    rospack = rospkg.RosPack()
    s = rospy.Service('classify', classify, handle_request)
    print("Classify server ready.")
    rospy.spin()



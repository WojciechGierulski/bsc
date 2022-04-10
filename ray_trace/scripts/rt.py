#!/usr/bin/env python3

import rospy
import rospkg
import os, sys
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

import numpy as np
from stl import mesh
import urdf_parser_py.urdf
from urdf_parser_py.urdf import URDF

def create_triangle(x, y, z):
    pts = []
    for i in range(0, 3):
        p = Point()
        p.x = x[i]
        p.y = y[i]
        p.z = z[i]
        pts.append(p)
    return pts

WANTED_LINKS = ["right_HandFingerThreeKnuckleTwoLink"]

def parse_xacro(rospack: rospkg.RosPack) -> urdf_parser_py.urdf.Robot:
    velma_desc_path = rospack.get_path('velma_description')
    ray_trace_path = rospack.get_path('ray_trace')
    os.system(f"xacro {velma_desc_path}/robots/velma.urdf.xacro > {ray_trace_path}/urdf/velma.urdf")
    robot = URDF.from_xml_file(f"{ray_trace_path}/urdf/velma.urdf")
    return robot


def get_link_stls(robot: urdf_parser_py.urdf.Robot, wanted_links: list, rospack: rospkg.RosPack) -> dict:
    stls = {}
    for link in robot.links:
        if link.name in wanted_links:
            stl_path = link.visual.geometry.filename
            stl_path = stl_path.split("://")[1]
            package_name = stl_path.split("/")[0]
            stl_path = "/".join(stl_path.split("/")[1:])
            package_path = rospack.get_path(package_name)
            stl_path = f"{package_path}/{stl_path}"
            stls[link.name] = stl_path
    return stls


if __name__ == "__main__":

    rospy.init_node('test_init', anonymous=False)
    rospack = rospkg.RosPack()
    robot = parse_xacro(rospack) #type: urdf_parser_py.urdf.Robot
    stls = get_link_stls(robot, WANTED_LINKS, rospack)

    cuboid = mesh.Mesh.from_file(stls["right_HandFingerThreeKnuckleTwoLink"])


    pub = rospy.Publisher('stll', MarkerArray, queue_size=10)
    rate = rospy.Rate(10)
    while True:
        markers = []
        for i, (xt, yt, zt) in enumerate(zip(cuboid.x, cuboid.y, cuboid.z)):
            m = Marker()
            m.id = i
            m.points = create_triangle(xt, yt, zt)
            m.pose.orientation.z = 1
            m.header.frame_id = "world"
            m.color.a = 255
            m.color.r = 255
            m.scale.x = 1
            m.scale.y = 1
            m.scale.z = 1
            m.type = 11
            markers.append(m)
        msg = MarkerArray()
        msg.markers = markers
        pub.publish(msg)
        rate.sleep()



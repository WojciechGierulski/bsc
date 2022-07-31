#!/usr/bin/env python3

import rospy
import rospkg
import os

from typing import List, Dict
import urdf_parser_py.urdf
from urdf_parser_py.urdf import URDF
import json
import csv

def get_wanted_links(rospack: rospkg.RosPack) -> List[str]:
    WANTED_LINKS = []
    with open(f"{rospack.get_path('ray_trace')}/stl_data/links.csv") as inputfile:
        for row in csv.reader(inputfile):
            if len(row) > 0:
                WANTED_LINKS.append(row[0])
    return WANTED_LINKS

def parse_xacro(rospack: rospkg.RosPack) -> urdf_parser_py.urdf.Robot:
    velma_desc_path = rospack.get_path('velma_description')
    ray_trace_path = rospack.get_path('ray_trace')
    os.system(f"xacro {velma_desc_path}/robots/velma.urdf.xacro > {ray_trace_path}/urdf/velma.urdf")
    robot = URDF.from_xml_file(f"{ray_trace_path}/urdf/velma.urdf")
    return robot


def get_link_stls(robot: urdf_parser_py.urdf.Robot, wanted_links: List[str], rospack: rospkg.RosPack) -> Dict[str, str]:
    stls_paths = {}
    stls_origins = {}
    for link in robot.links:
        if link.name in wanted_links:
            for i, l_visual in enumerate(link.visuals):
                if hasattr(l_visual.geometry, "filename"): # this is stl
                    stl_path = l_visual.geometry.filename
                    stl_path = stl_path.split("://")[1]
                    package_name = stl_path.split("/")[0]
                    stl_path = "/".join(stl_path.split("/")[1:])
                    package_path = rospack.get_path(package_name)
                    stl_path = f"{package_path}/{stl_path}"
                    stls_paths[link.name + f"_{i}"] = stl_path
                    stls_origins[link.name + f"_{i}"] = {"xyz": l_visual.origin.xyz, "rpy": l_visual.origin.rpy}
                    if link.name == "right_arm_5_link":
                        break
    return stls_paths, stls_origins



if __name__ == "__main__":

    rospy.init_node('dump_stl', anonymous=False)
    rospack = rospkg.RosPack()
    WANTED_LINKS = get_wanted_links(rospack)
    robot = parse_xacro(rospack) #type: urdf_parser_py.urdf.Robot
    stls_paths, stls_origins = get_link_stls(robot, WANTED_LINKS, rospack)


    stl_paths_file = open(f"{rospack.get_path('ray_trace')}/stl_data/stl_paths.json", "w")
    stl_paths_file.seek(0)
    stl_paths_file.truncate()
    json.dump(stls_paths, stl_paths_file, indent=2)
    stl_paths_file.close()

    stl_paths_file = open(f"{rospack.get_path('ray_trace')}/stl_data/stl_origins.json", "w")
    stl_paths_file.seek(0)
    stl_paths_file.truncate()
    json.dump(stls_origins, stl_paths_file, indent=2)
    stl_paths_file.close()
    print("end")


import sys

import rospy
from rcprg_ros_utils import exitError
from velma_common import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
from rcprg_ros_utils import exitError
from rcprg_planner import *
import math
import json


class Initializer:
    @staticmethod
    def initialize_system():


        velma = VelmaInterface()

        if not velma.waitForInit(timeout_s=10.0):
            exitError(1)


        if velma.enableMotors() != 0:
            exitError(14)

        velma.startHomingHP()
        if velma.waitForHP() != 0:
            exitError(14)

        velma.startHomingHT()
        if velma.waitForHT() != 0:
            exitError(15)

        print("Motors must be enabled every time after the robot enters safe state.")
        print("If the motors are already enabled, enabling them has no effect.")
        print("Enabling motors...")
        if velma.enableMotors() != 0:
            exitError(2)

        print("Moving to the current position...")
        js_start = velma.getLastJointState()
        velma.moveJoint(js_start[1], 0.5, start_time=0.5, position_tol=15.0 / 180.0 * math.pi)
        error = velma.waitForJoint()
        if error != 0:
            print("The action should have ended without error, but the error code is", error)
            exitError(3)

        return velma

    @staticmethod
    def get_planner(velma):
        planner = Planner(velma.maxJointTrajLen())
        if planner.waitForInit():
            return planner
        else:
            print("Could not initialize planner")
            exitError(99)

    @staticmethod
    def get_octomap():
        oml = OctomapListener("/octomap_binary")
        rospy.sleep(1.0)
        octomap = oml.getOctomap(timeout_s=5.0)
        return octomap

    @staticmethod
    def process_octomap(planner, octomap):
        if not planner.processWorld(octomap):
            print("Processing world failed")
            exitError(99)

    @staticmethod
    def get_solver():
        return KinematicsSolverVelma()

    @staticmethod
    def _load_init_pos(rt_path):
        with open(rt_path+"/config/init_pose.json", "r") as file:
            pose = json.load(file)
            return  pose

    @staticmethod
    def move_to_init_pose(velma, rt_path):
        pose = dict(Initializer._load_init_pos(rt_path))
        velma.moveJoint(pose, 5.0)
        error = velma.waitForJoint()
        if error != 0:
            print error
            sys.exit()
        rospy.sleep(0.1)

        q_dest = (pose["head_pan_joint"], pose["head_tilt_joint"])
        velma.moveHead(q_dest, 2.0, start_time=0.1)
        error = velma.waitForHead()
        if error != 0:
            print error
            sys.exit()
        rospy.sleep(0.1)


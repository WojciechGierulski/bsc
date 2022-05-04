import rospy
from rcprg_ros_utils import exitError
from velma_common import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
from rcprg_ros_utils import exitError
from rcprg_planner import *
import math


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

        print("moving head to position: 0")
        q_dest = (0, 0)
        velma.moveHead(q_dest, 3.0, start_time=0.5)
        if velma.waitForHead() != 0:
            exitError(4)
        rospy.sleep(0.5)
        if not isHeadConfigurationClose(velma.getHeadCurrentConfiguration(), q_dest, 0.1):
            exitError(5)
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


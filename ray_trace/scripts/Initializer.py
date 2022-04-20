import rospy
from rcprg_ros_utils import exitError
from velma_common import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
from rcprg_ros_utils import exitError
from rcprg_planner import *


class Initializer:
    @staticmethod
    def initialize_system():

        print("Running python interface for Velma...")
        velma = VelmaInterface()
        print("Waiting for VelmaInterface initialization...")
        if not velma.waitForInit(timeout_s=10.0):
            print("Could not initialize VelmaInterface\n")
            exitError(1)
        print("Initialization ok!\n")

        print("Motors must be enabled every time after the robot enters safe state.")
        print("If the motors are already enabled, enabling them has no effect.")
        print("Enabling motors...")
        if velma.enableMotors() != 0:
            exitError(14)

        print("Also, head motors must be homed after start-up of the robot.")
        print("Sending head pan motor START_HOMING command...")
        velma.startHomingHP()
        if velma.waitForHP() != 0:
            exitError(14)
        print("Head pan motor homing successful.")

        print("Sending head tilt motor START_HOMING command...")
        velma.startHomingHT()
        if velma.waitForHT() != 0:
            exitError(15)
        print("Head tilt motor homing successful.")
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


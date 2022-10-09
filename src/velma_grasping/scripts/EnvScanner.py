from velma_common import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
from rcprg_ros_utils import exitError
from rcprg_planner import *
import math


class EnvScanner:

    # Positions
    torso_center = symmetricalConfiguration({'torso_0_joint': 0.0,})
    torso_left = symmetricalConfiguration({'torso_0_joint': -1.05,})
    torso_right = symmetricalConfiguration({'torso_0_joint': 1.05,})
    torso_leftleft = symmetricalConfiguration({'torso_0_joint': -1.3,})
    torso_rightright = symmetricalConfiguration({'torso_0_joint': 1.3,})
    head_center = (0, 0)
    head_down = (0, 1.0)
    head_up = (0, -0.75)

    @staticmethod
    def move_torso(pos, time, velma):
        velma.moveJoint(pos, time, start_time=0.5, position_tol=10.0/180.0*math.pi)
        error = velma.waitForJoint()
        if error != 0:
            exitError(2, msg="The action should have ended without error,"\
                            " but the error code is {}".format(error))

    @staticmethod
    def move_head(pos, time, velma):
        velma.moveHead(pos, time, start_time=0.5)
        if velma.waitForHead() != 0:
            exitError(14)
        if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), pos, 0.1 ):
            exitError(15)

    @staticmethod
    def scan_env(velma):
        print('Movint to starting position')
        EnvScanner.move_torso(EnvScanner.torso_center, 2.0, velma)
        print('Scanning center')
        EnvScanner.move_head(EnvScanner.head_up, 2.0, velma)
        EnvScanner.move_head(EnvScanner.head_down, 4.0, velma)
        EnvScanner.move_head(EnvScanner.head_center, 2.0, velma)
        print('Moving to left')
        EnvScanner.move_torso(EnvScanner.torso_left, 2.0, velma)
        print('Scanning left')
        EnvScanner.move_head(EnvScanner.head_up, 2.0, velma)
        EnvScanner.move_head(EnvScanner.head_down, 4.0, velma)
        EnvScanner.move_head(EnvScanner.head_center, 2.0, velma)
        print('Moving to left-left')
        EnvScanner.move_torso(EnvScanner.torso_leftleft, 1.0, velma)
        EnvScanner.move_head((-1.0, 0), 2.0, velma)
        print("Scanning left-left")
        EnvScanner.move_head((-1, -0.75), 2.0, velma)
        EnvScanner.move_head((-1, 1), 4.0, velma)
        EnvScanner.move_head(EnvScanner.head_center, 2.0, velma)
        print('Moving to right')
        EnvScanner.move_torso(EnvScanner.torso_right, 4.0, velma)
        print('Scanning right')
        EnvScanner.move_head(EnvScanner.head_up, 2.0, velma)
        EnvScanner.move_head(EnvScanner.head_down, 4.0, velma)
        EnvScanner.move_head(EnvScanner.head_center, 2.0, velma)
        print('Moving right-right')
        EnvScanner.move_torso(EnvScanner.torso_rightright, 1.0, velma)
        EnvScanner.move_head((1.0, 0), 2.0, velma)
        print('Scanning right-right')
        EnvScanner.move_head((1, -0.75), 2.0, velma)
        EnvScanner.move_head((1, 1), 4.0, velma)
        EnvScanner.move_head(EnvScanner.head_center, 2.0, velma)
        print('Moving to starting position')
        EnvScanner.move_torso(EnvScanner.torso_center, 2.0, velma)
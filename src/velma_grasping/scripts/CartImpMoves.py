from velma_common import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
from rcprg_ros_utils import exitError
from rcprg_planner import *
import math
import PyKDL
import rospy


class CartImpMoves:

    @staticmethod
    def switch_to_cartimp(velma):
        print ("Switch to cart_imp mode (no trajectory)...")
        if not velma.moveCartImpRightCurrentPos(start_time=0.2):
            exitError(8)
        if velma.waitForEffectorRight() != 0:
            exitError(9)

        rospy.sleep(0.5)

        diag = velma.getCoreCsDiag()
        if not diag.inStateCartImp():
            print ("The core_cs should be in cart_imp state, but it is not")
            exitError(3)

    @staticmethod
    def move_gripper_forward(velma, hand, solver, dX=0.2):
        if hand == 'right':
            time, velma_pos = velma.getLastJointState()
            configuration = (
            velma_pos['right_arm_0_joint'], velma_pos['right_arm_1_joint'], velma_pos['right_arm_2_joint'],
            velma_pos['right_arm_3_joint'], velma_pos['right_arm_4_joint'], velma_pos['right_arm_5_joint'],
            velma_pos['right_arm_6_joint'])
            frame = solver.getRightArmFk(velma_pos['torso_0_joint'], configuration)
            p = PyKDL.Vector(dX, 0, 0)
            p = frame * p
            frame.p = p
            if not velma.moveCartImpRight([frame], [2.0], None, None, None, None,
                                          PyKDL.Wrench(PyKDL.Vector(5, 5, 5), PyKDL.Vector(5, 5, 5)), start_time=0.5):
                exitError(16)
            if velma.waitForEffectorRight() != 0:
                exitError(17)
        if hand == 'left':
            time, velma_pos = velma.getLastJointState()
            configuration = (
            velma_pos['left_arm_0_joint'], velma_pos['left_arm_1_joint'], velma_pos['left_arm_2_joint'],
            velma_pos['left_arm_3_joint'], velma_pos['left_arm_4_joint'], velma_pos['left_arm_5_joint'],
            velma_pos['left_arm_6_joint'])
            frame = solver.getLeftArmFk(velma_pos['torso_0_joint'], configuration)
            p = PyKDL.Vector(dX, 0, 0)
            p = frame * p
            frame.p = p
            if not velma.moveCartImpLeft([frame], [2.0], None, None, None, None,
                                         PyKDL.Wrench(PyKDL.Vector(5, 5, 5), PyKDL.Vector(5, 5, 5)), start_time=0.5):
                exitError(16)
            if velma.waitForEffectorLeft() != 0:
                exitError(17)

    @staticmethod
    def move_gripper_up(velma, hand, solver, dZ=0.3):
        if hand == 'right':
            time, velma_pos = velma.getLastJointState()
            configuration = (
            velma_pos['right_arm_0_joint'], velma_pos['right_arm_1_joint'], velma_pos['right_arm_2_joint'],
            velma_pos['right_arm_3_joint'], velma_pos['right_arm_4_joint'], velma_pos['right_arm_5_joint'],
            velma_pos['right_arm_6_joint'])
            frame = solver.getRightArmFk(velma_pos['torso_0_joint'], configuration)
            p = PyKDL.Vector(0, 0, dZ)
            p = frame * p
            frame.p = p
            if not velma.moveCartImpRight([frame], [2.0], None, None, None, None,
                                          PyKDL.Wrench(PyKDL.Vector(5, 5, 5), PyKDL.Vector(5, 5, 5)), start_time=0.5):
                exitError(16)
            if velma.waitForEffectorRight() != 0:
                exitError(17)
        if hand == 'left':
            time, velma_pos = velma.getLastJointState()
            configuration = (
            velma_pos['left_arm_0_joint'], velma_pos['left_arm_1_joint'], velma_pos['left_arm_2_joint'],
            velma_pos['left_arm_3_joint'], velma_pos['left_arm_4_joint'], velma_pos['left_arm_5_joint'],
            velma_pos['left_arm_6_joint'])
            frame = solver.getLeftArmFk(velma_pos['torso_0_joint'], configuration)
            p = PyKDL.Vector(0, 0, dZ)
            p = frame * p
            frame.p = p
            if not velma.moveCartImpLeft([frame], [2.0], None, None, None, None,
                                         PyKDL.Wrench(PyKDL.Vector(5, 5, 5), PyKDL.Vector(5, 5, 5)), start_time=0.5):
                exitError(16)
            if velma.waitForEffectorLeft() != 0:
                exitError(17)
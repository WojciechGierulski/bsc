import rospy
from JointImpMoves import JointImpMoves
from CartImpMoves import CartImpMoves
from GripperMoves import GripperMoves
import copy

class SequenceExecutor():
    def __init__(self):
        pass

    @staticmethod
    def execute_sequence(seq, velma, hand, solver):
        max_i = len(seq)
        i = 0
        while i < max_i:
            if seq[i] == "gripper":
                GripperMoves.move_grippers(velma, hand, seq[i+1])
                i += 1
            elif seq[i] == "forward":
                CartImpMoves.move_gripper_forward(velma, hand, solver, seq[i+1])
                i += 1
            elif seq[i] == "up":
                CartImpMoves.move_gripper_up(velma, hand, solver, seq[i+1])
                i += 1
            i += 1
            if i == max_i - 1:
                break
            rospy.sleep(0.5)

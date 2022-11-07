from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError
import rospy
import math
import numpy as np
import json
import sys

class JointImpMoves:

    @staticmethod
    def switch_to_jimp(velma):
        print ("Switch to jnt_imp mode (no trajectory)...")
        velma.moveJointImpToCurrentPos(start_time=0.2)
        error = velma.waitForJoint()
        if error != 0:
            print ("The action should have ended without error, but the error code is", error)
            exitError(3)
        rospy.sleep(0.5)
        diag = velma.getCoreCsDiag()
        if not diag.inStateJntImp():
            print ("The core_cs should be in jnt_imp state, but it is not")
            exitError(3)


    @staticmethod
    def move_with_planning(velma, qs, planner, collision_object=None):
        qs = [qMapToConstraints(q, 0.02, group=velma.getJointGroup("impedance_joints")) for q in qs]
        for i in range(15):
            rospy.sleep(0.1)
            js = velma.getLastJointState()
            print("Planning (try", i, ")...")
            if collision_object is not None:
                traj = planner.plan(js[1], qs, "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect",
                                    attached_collision_objects=[collision_object])
            elif collision_object is None:
                traj = planner.plan(js[1], qs, "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
            if traj == None:
                continue
            print("Executing trajectory...")
            if not velma.moveJointTraj(traj, start_time=0.5, position_tol=11.0/180.0 * math.pi, velocity_tol=11.0/180.0*math.pi):
                exitError(5)
            if velma.waitForJoint() == 0:
                return True
            else:
                print("The trajectory could not be completed, retrying...")
                continue
        return False


    @staticmethod
    def get_IK_for_frames(hand, frames, solver):
        # IK - inverse kinematics
        IKs = []
        torsos = []
        for torso in np.linspace(-1.55, 1.55, 6):
            for elbow_ang in np.linspace(-1.55, 1.55, 6):
                for frame in frames:
                    ik = solver.calculateIkArm(hand, frame, torso, elbow_ang, False, False, False)
                    if None not in ik:
                        IKs.append(ik)
                        torsos.append(torso)
        return IKs, torsos

    @staticmethod
    def transform_iks_and_torsos_to_q(IKs, torsos, hand):
        # transform ik solution to dictionary {joint_name: value}
        qs = []
        if hand == 'right':
            for torso, IK in zip(torsos, IKs):
                qs.append(JointImpMoves.get_joint_dict_right_arm(IK, torso))
        elif hand == 'left':
            for torso, IK in zip(torsos, IKs):
                qs.append(JointImpMoves.get_joint_dict_left_arm(IK, torso))
        return qs

    @staticmethod
    def get_joint_dict_right_arm(IK, torso):
        q_map_goal = {'torso_0_joint':torso, 'right_arm_0_joint':IK[0], 'right_arm_1_joint':IK[1],
            'right_arm_2_joint':IK[2], 'right_arm_3_joint':IK[3], 'right_arm_4_joint':IK[4], 'right_arm_5_joint':IK[5],
            'right_arm_6_joint':IK[6]}
        return q_map_goal

    @staticmethod
    def get_joint_dict_left_arm(IK, torso):
        q_map_goal = {'torso_0_joint':torso, 'left_arm_0_joint':IK[0], 'left_arm_1_joint':IK[1],
            'left_arm_2_joint':IK[2], 'left_arm_3_joint':IK[3], 'left_arm_4_joint':IK[4], 'left_arm_5_joint':IK[5],
            'left_arm_6_joint':IK[6]}
        return q_map_goal

    @staticmethod
    def move_to_init_pos(velma):
        q = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
            'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
            'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
            'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
        velma.moveJoint(q, 4.0)
        error = velma.waitForJoint()
        if error != 0:
            print(error)
            sys.exit()
        rospy.sleep(0.1)

    @staticmethod
    def _load_calib_pos(rt_path, calib_pose_nr):
        with open(rt_path+"/config/init_pose_"+str(calib_pose_nr)+".json", "r") as file:
            pose = json.load(file)
            return pose

    @staticmethod
    def move_to_calib_pose(velma, planner, rt_path, calib_pose_nr):
        pose = dict(JointImpMoves._load_calib_pos(rt_path, calib_pose_nr))
        JointImpMoves.move_with_planning(velma, [pose], planner)

        q_dest = (0, 0.72)
        velma.moveHead(q_dest, 2.0, start_time=0.1)
        error = velma.waitForHead()
        if error != 0:
            print(error)
            sys.exit()
        rospy.sleep(0.1)
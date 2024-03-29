from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError
import rospy
import math
import numpy as np
import json
import sys
from moveit_msgs.msg import Constraints, JointConstraint

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
    def move_with_planning(velma, qs, planner, hand, collision_object=None):
        #qs = [qMapToConstraints(q, 0.01, group=velma.getJointGroup("impedance_joints")) for q in qs]
        if hand == "right":
            qs = JointImpMoves.get_left_arm_constraints(qs)
        elif hand == "left":
            qs = JointImpMoves.get_right_arm_constraints(qs)
        else:
            qs = [qMapToConstraints(q, 0.01, group=velma.getJointGroup("impedance_joints")) for q in qs]
        traj = None
        for i in range(10):
            rospy.sleep(0.1)
            js = velma.getLastJointState()
            print("Planning (try", i, ")...")
            if collision_object is not None:
                traj = planner.plan(js[1], qs, "impedance_joints", num_planning_attempts=5, max_velocity_scaling_factor=1, planner_id="RRTConnect",
                                    attached_collision_objects=[collision_object])
            elif collision_object is None:
                traj = planner.plan(js[1], qs, "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15,
                                    max_acceleration_scaling_factor=0.15, planner_id="RRTConnect")
            if traj is None:
                continue
            print("Executing trajectory...")
            if not velma.moveJointTraj(traj, start_time=0.5, position_tol=15/180.0*math.pi, velocity_tol=15/180.0*math.pi):
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
        for torso in np.linspace(-1.5, 1.5, 10):
            for elbow_ang in np.linspace(-1.5, 1.5, 10):
                for frame in frames:
                    ik1 = solver.calculateIkArm(hand, frame, torso, elbow_ang, False, False, False)
                    if None not in ik1:
                        IKs.append(ik1)
                        torsos.append(torso)

        return IKs, torsos

    @staticmethod
    def transform_iks_and_torsos_to_q(IKs, torsos, hand):
        # transform ik solution to dictionary {joint_name: value}
        qs = []
        if hand == 'right':
            for torso, IK in zip(torsos, IKs):
                q = JointImpMoves.get_joint_dict_right_arm(IK, torso)
                #q.update(JointImpMoves.get_left_arm_init_dict())
                qs.append(q)
        elif hand == 'left':
            for torso, IK in zip(torsos, IKs):
                q = JointImpMoves.get_joint_dict_left_arm(IK, torso)
                #q.update(JointImpMoves.get_right_arm_init_dict())
                qs.append(q)
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
        JointImpMoves.move_head((0,0), velma)
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
    def get_left_arm_init_dict():
        return {'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
            'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

    @staticmethod
    def get_right_arm_init_dict():
        return {'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
            'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
            'right_arm_6_joint':0}

    @staticmethod
    def _load_calib_pos(rt_path, calib_pose_nr):
        with open(rt_path+"/config/init_pose_"+str(calib_pose_nr)+".json", "r") as file:
            pose = json.load(file)
            return pose

    @staticmethod
    def move_to_calib_pose(velma, planner, rt_path, calib_pose_nr):
        pose = dict(JointImpMoves._load_calib_pos(rt_path, calib_pose_nr))
        JointImpMoves.move_with_planning(velma, [pose], planner, None)

        q_dest = (0, 0.72)
        velma.moveHead(q_dest, 2.0, start_time=0.1)
        error = velma.waitForHead()
        if error != 0:
            print(error)
            sys.exit()
        rospy.sleep(0.1)

    @staticmethod
    def move_head(q_dest, velma):
        velma.moveHead(q_dest, 2.0, start_time=0.1)
        error = velma.waitForHead()
        if error != 0:
            print(error)
            sys.exit()
        rospy.sleep(0.1)

    @staticmethod
    def gripper_to_joint_7_transform(velma, frames, hand):
        # Transform gripper frames to 7th joint frames
        new_frames = []
        if hand == 'right':
            transformation = velma.getTf('Gr', 'Wr')
        elif hand == 'left':
            transformation = velma.getTf('Gl', 'Wl')
        for frame in frames:
            new_frames.append(frame*transformation)
        return new_frames

    @staticmethod
    def get_left_arm_constraints(qs):
        total_qs = []
        c_dict = JointImpMoves.get_left_arm_init_dict()
        for q in qs:
            curr_c = Constraints()
            for joint_name, joint_value in q.items():
                curr_c.joint_constraints.append(JointConstraint(joint_name, joint_value, 0.01, 0.01, 1.0))
            for joint_name, joint_value in c_dict.items():
                curr_c.joint_constraints.append(JointConstraint(joint_name, joint_value, 0.02, 0.02, 1.0))
            total_qs.append(curr_c)
        return total_qs

    @staticmethod
    def get_right_arm_constraints(qs):
        total_qs = []
        c_dict = JointImpMoves.get_right_arm_init_dict()
        for q in qs:
            curr_c = Constraints()
            for joint_name, joint_value in c_dict.items():
                curr_c.joint_constraints.append(JointConstraint(joint_name, joint_value, 0.02, 0.02, 1.0))
            for joint_name, joint_value in q.items():
                curr_c.joint_constraints.append(JointConstraint(joint_name, joint_value, 0.01, 0.01, 1.0))
            total_qs.append(curr_c)
        return total_qs


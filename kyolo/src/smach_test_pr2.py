#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_plugs_actions')

import rospy

import threading
import copy

import actionlib
import PyKDL

from pr2_plugs_msgs.msg import *
from pr2_plugs_msgs.srv import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *
from pr2_common_action_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal

from std_srvs.srv import *

from tf_conversions.posemath import fromMsg, toMsg
from pr2_arm_move_ik.tools import *

# State machine classes
import smach
from smach import *
from smach_ros import *

from detect_outlet import construct_sm as construct_detect_outlet_sm
from fetch_plug import construct_sm as construct_fetch_plug_sm
from plug_in import construct_sm as construct_plug_in_sm

from pr2_plugs_actions.tf_util import TFUtil

def main():
    rospy.init_node("recharge_toplevel")
    TFUtil()

    # Construct state machine
    recharge_sm = StateMachine(
            outcomes=['plugged_in','unplugged','aborted','preempted'],
            input_keys = ['recharge_command'],
            output_keys = ['recharge_state'])

    # Set the initial state explicitly
    recharge_sm.set_initial_state(['PROCESS_RECHARGE_COMMAND'])
    recharge_sm.userdata.recharge_state = RechargeState(state=RechargeState.UNPLUGGED)

    recharge_sm.userdata.base_to_outlet        = Pose(position=Point(-0.0178, -0.7474,  0.2824), orientation=Quaternion( 0.0002, -0.0002, -0.7061, 0.7081))
    recharge_sm.userdata.gripper_to_plug_grasp = Pose(position=Point( 0.0282, -0.0050, -0.0103), orientation=Quaternion(-0.6964,  0.1228,  0.1228, 0.6964))
    recharge_sm.userdata.base_to_plug_on_base  = Pose(position=Point( 0.0783,  0.0244,  0.2426), orientation=Quaternion( 0.4986,  0.4962,  0.4963, 0.5088))
    recharge_sm.userdata.gripper_to_plug       = Pose(position=Point( 0.0275, -0.0046, -0.0094), orientation=Quaternion(-0.6876,  0.1293,  0.1226, 0.7039))

    with recharge_sm:
        ### PLUGGING IN ###
        @smach.cb_interface(input_keys=['recharge_command'])
        def plug_in_cond(ud):
            command = ud.recharge_command.command
            if command is RechargeCommand.PLUG_IN:
                return True
            elif command is RechargeCommand.UNPLUG:
                return False
        StateMachine.add('PROCESS_RECHARGE_COMMAND',
                ConditionState(cond_cb = plug_in_cond),
                { 'true':'NAVIGATE_TO_OUTLET',
                    'false':'UNPLUG'})
        

        
        sm_nav = StateMachine(
                outcomes=['succeeded','aborted','preempted'],
                input_keys = ['recharge_command'])
        StateMachine.add('NAVIGATE_TO_OUTLET', sm_nav,
                {'succeeded':'DETECT_OUTLET',
                    'aborted':'FAIL_STILL_UNPLUGGED'})
        with sm_nav:
            StateMachine.add('GOAL_IS_LOCAL', 
                    ConditionState(
                        cond_cb = lambda ud: ud.recharge_command.plug_id == 'local',
                        input_keys = ['recharge_command']),
                    {'true': 'UNTUCK_AT_OUTLET',
                        'false': 'SAFETY_TUCK'})
            StateMachine.add('SAFETY_TUCK', 
                    SimpleActionState('tuck_arms', TuckArmsAction,
                        goal = TuckArmsGoal(True,True)),
                    { 'succeeded':'GET_OUTLET_LOCATIONS',
                      'aborted':'SAFETY_TUCK'})
            StateMachine.add('GET_OUTLET_LOCATIONS',
                    ServiceState('outlet_locations', GetOutlets,
                        response_slots=['poses']),
                    {'succeeded':'NAVIGATE'},
                    remapping={'poses':'approach_poses'})

            @smach.cb_interface(input_keys=['approach_poses','recharge_command'])
            def get_outlet_approach_goal(ud,goal):
                """Get the approach pose from the outlet approach poses list"""

                # Get id from command
                plug_id = ud.recharge_command.plug_id

                # Grab the relevant outlet approach pose
                for outlet in ud.approach_poses:
                    if outlet.name == plug_id or outlet.id == plug_id:
                        target_pose = PoseStamped(pose=outlet.approach_pose)

                # Create goal for move base
                move_base_goal = MoveBaseGoal()
                move_base_goal.target_pose = target_pose
                move_base_goal.target_pose.header.stamp = rospy.Time.now()
                move_base_goal.target_pose.header.frame_id = "map"

                return move_base_goal



                
            StateMachine.add('NAVIGATE', 
                             SimpleActionState('pr2_move_base',MoveBaseAction,
                                               goal_cb=get_outlet_approach_goal,
                                               exec_timeout = rospy.Duration(20*60.0)),
                             { 'succeeded':'UNTUCK_AT_OUTLET' })
            StateMachine.add('UNTUCK_AT_OUTLET', 
                             SimpleActionState('tuck_arms', TuckArmsAction,
                                               goal = TuckArmsGoal(False, False)))

        StateMachine.add('DETECT_OUTLET', 
                         SimpleActionState('detect_outlet',DetectOutletAction,
                                           result_slots = ['base_to_outlet_pose']),
                         {'succeeded':'FETCH_PLUG',
                          'aborted':'FAIL_STILL_UNPLUGGED'},
                         remapping = {'base_to_outlet_pose':'base_to_outlet'})

        StateMachine.add('FETCH_PLUG',
                         SimpleActionState('fetch_plug',FetchPlugAction,
                                           result_slots = ['plug_on_base_pose', 'gripper_plug_grasp_pose']),
                         {'succeeded':'PLUG_IN',
                          'aborted':'FAIL_OPEN_GRIPPER'},
                         remapping = {'plug_on_base_pose':'base_to_plug_on_base', 'gripper_plug_grasp_pose':'gripper_to_plug_grasp'})
        
        @smach.cb_interface(input_keys=['recharge_state'], output_keys=['recharge_state'])
        def set_plug_in_result(ud, result_status, result):
            if result_status == GoalStatus.SUCCEEDED:
                ud.recharge_state.state = RechargeState.PLUGGED_IN
        StateMachine.add('PLUG_IN',
                         SimpleActionState('plug_in',PlugInAction,
                                           goal_slots = ['base_to_outlet'],
                                           result_slots = ['gripper_to_plug'],
                                           result_cb = set_plug_in_result),
                         { 'succeeded':'STOW_LEFT_ARM',
                           'aborted':'RECOVER_STOW_PLUG'})
        
        # Move L arm out of the way
        StateMachine.add('STOW_LEFT_ARM',
                         SimpleActionState('l_arm_controller/joint_trajectory_action', JointTrajectoryAction,
                                           goal = stow_l_arm_goal),
                         { 'succeeded':'plugged_in'})

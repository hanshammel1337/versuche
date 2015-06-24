#!/usr/bin/env python

## ROS ##
import rospy
import tf
import numpy as np
from math import sqrt
import pitasc_core.kinematics as kin

## piTaSC ##
from pitasc_core.robots.robot_lbr4 import Robot_LBR4

from pitasc_core.chains.chain_simple_pose import Chain_SimplePose

from pitasc_core.kinematic_loop import KinematicLoop

from solver_joint_middle_v import Solver_Jointmiddle
from solver_prioritized_versucha import Solver_Prioritized

from pitasc_core.controllers.controller_simple import Controller_P

from pitasc_core.task import Task
from scene_simple_versuchc import Scene_Simple


## ROS node ##
## -----------------------------------------------------------------##
def run():
	##---------ROS ##
	rospy.init_node('pitasc')

	#########################################################################################
	##---------Virtual Kinematic Chains ##
	# SYNTAX:        Chain_SimplePose(self, name, prefix, variable_type, from_frame, to_frame)
	# integrator=True  => dq's werden ueber sensor_msgs.msg/JointState gepublished
	# integrator=False => JointTrajectory wird zum roboter gesendet
	print 'Setting up the kinematic chains'
	robot = Robot_LBR4("lbr4", use_integrator=False)

	# Chain_SimplePose uses "Driver_Tflistener"
	target_to_eef  = Chain_SimplePose('target_to_eef', 'x1', 'feature', 'moving_tf', 'tool_center')
	base_to_target = Chain_SimplePose('base_to_target', 'o1', 'object', 'base_link', 'moving_tf')
	
	#########################################################################################
	##---------Kinematic Loops ##
	# SYNTAX: KinematicLoop(self, name, robot_chains, feature_chains):
	print 'Setting up the kinematic loops'
	loops = {}
	# SYNTAX: KinematicLoop(self, name, robot_chains, feature_chains):
	loops['l1'] = KinematicLoop('l1',
		robot_chains=[robot.chains[0]],
		feature_chains=[base_to_target.chains[0], target_to_eef.chains[0]]
	)
	#########################################################################################
	##---------Tasks ##
	# SYNTAX Task(self, symbols, desired, controllers = []):
	task = []
	task.append( Task( target_to_eef.chains[0].symbols[0:6] ,  [0,0,0, 0,0,0] , []) )
	#task.append( Task( target_to_eef.chains[0].symbols[0:6] ,  [0,0,0,0,0,0] , []) )
	#task.append( Task( robot.chains[0].symbols[0:7] ,  [0,0,0,0,0,0,0] , []) )

	#########################################################################################
	##---------Controller ##
	p_controller = Controller_P(0.3)

	#########################################################################################
	##---------Scene ##
	print 'Setting up the scene'
	scene = Scene_Simple()
	scene.versuchsname = 'middle'
	#scene.versuchsname = 'free'
	#solver = Solver_Prioritized(scene)           #__TOGGLE
	solver = Solver_Jointmiddle(scene)            #__TOGGLE

	scene.default_controller = p_controller
	scene.solver = solver
	
	scene.add_drivers(target_to_eef.drivers)
	scene.add_drivers(base_to_target.drivers)
	scene.add_robot_drivers(robot.robot_drivers)

	scene.add_links(robot.links + target_to_eef.links + base_to_target.links)
	scene.add_chains(robot.chains + target_to_eef.chains + base_to_target.chains)
	scene.add_loops(loops)

	scene.build_symbols()
	scene.set_tasks(task)
	
	robot.robot_drivers[0].set_observer(scene.update)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
	#########################################################################################
	##---------ROS loop
	#while not rospy.is_shutdown():
	#	scene.update()


## Main
## -----------------------------------------------------------------##
if __name__ == '__main__':
	try:
		## If this file has been executed, run it ##
		run()
	except rospy.ROSInterruptException:
		pass

# eof
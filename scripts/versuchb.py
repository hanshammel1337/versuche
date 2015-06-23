#!/usr/bin/env python

## ROS ##
import rospy
import tf

## piTaSC ##
from pitasc_core.robots.robot_lbr4 import Robot_LBR4
from pitasc_core.robots.robot_ur5 import Robot_UR5

from pitasc_core.chains.chain_simple_pose import Chain_SimplePose
from pitasc_core.kinematic_loop import KinematicLoop
from pitasc_core.controllers.controller_simple import Controller_P
from pitasc_core.task import Task


#from pitasc_core.scene_simple_versuchb import Scene_Simple_Obstacle
#from pitasc_core.solvers.solver_prioritized_v import Solver_Prioritized_Active
from scene_simple_versuchb import Scene_Simple_Obstacle
from solver_prioritized_v import Solver_Prioritized_Active

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
	robot = Robot_LBR4("", use_integrator=True)
	
	# Chain_SimplePose uses "Driver_Tflistener"
	target_to_eef  = Chain_SimplePose('target_to_eef', 'x1', 'feature', 'target', 'tool_center')
	obstacle_to_eef = Chain_SimplePose('obstacle_to_eef', 'x2', 'feature', 'obstacle_1', 'tool_center')

	base_to_target = Chain_SimplePose('base_to_target', 'o1', 'object', 'base_link', 'target')
	base_to_obstacle = Chain_SimplePose('base_to_obstacle','o2', 'object', 'base_link', 'obstacle_1')
	
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
	
	loops['l2'] = KinematicLoop('l2',
		robot_chains=[robot.chains[0]],
		feature_chains=[base_to_obstacle.chains[0], obstacle_to_eef.chains[0]]
	)
	
	#########################################################################################
	##---------Tasks ##
	# SYNTAX Task(self, symbols, desired, controllers = []):
	task = []
	#task.append( Task( obstacle_to_eef.chains[0].symbols[2:3], [0.13]         , []) )  #FUNKTIONIERT
	task.append( Task( obstacle_to_eef.chains[0].symbols[1:2], [0.13]         , []) )
	task.append( Task( target_to_eef.chains[0].symbols[0:6] ,  [0,0,0, 0,0,0] , []) )

	#########################################################################################
	##---------Controller ##
	p_controller = Controller_P(0.3)

	##---------Solver ##
	solver = Solver_Prioritized_Active()
	#solver = Solver_LeastSquare()

	#########################################################################################
	##---------Scene ##
	print 'Setting up the scene'
	scene = Scene_Simple_Obstacle()

	scene.default_controller = p_controller
	scene.solver = solver
	scene.start_point = [0.15,-0.1,0.4 ,0,3.1415,0]
	scene.end_point = [0.6,-0.3,0.1 ,0,3.1415,0]
	#scene.start_point = [0.41,-0.18,0 ,0,0,0]		# another point
	#scene.end_point = [0.41,-0.18,0.5 ,0,0,0]
	scene.send_target(scene.start_point)
	
	scene.add_drivers(target_to_eef.drivers)
	scene.add_drivers(obstacle_to_eef.drivers)
	scene.add_drivers(base_to_target.drivers)
	scene.add_drivers(base_to_obstacle.drivers)
	scene.add_robot_drivers(robot.robot_drivers)

	scene.add_links(robot.links + target_to_eef.links + obstacle_to_eef.links + base_to_target.links + base_to_obstacle.links)
	scene.add_chains(robot.chains + target_to_eef.chains + obstacle_to_eef.chains + base_to_target.chains + base_to_obstacle.chains)
	scene.add_loops(loops)

	scene.build_symbols()
	scene.set_initial_tasks(task)
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
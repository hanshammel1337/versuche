#!/usr/bin/env python

## Python ##
##
import numpy as np
from math import sqrt, atan2
import time, string, getpass

import threading

from pitasc_core.scene_simple import Scene_Simple
from pitasc_core.pitasc import PiTaSC
import tf
import rospy

class Scene_Simple_Obstacle(Scene_Simple):
    
    def __init__(self):
        """ 
            Scene_Simple with obstacle avoidance
        """
        super(Scene_Simple_Obstacle, self).__init__()

        self.initial_tasks =[]

        self.obstacle_distance = 0.0
        self.target_distance = 1.0
        self.target_ee_x = None
        self.target_ee_y = None
        self.target_ee_z = None

        self.target_base_x = None
        self.target_base_y = None
        self.target_base_z = None

        self.active = 1.0
        self.wievieleaufgabeaktiv = 0
        self.start_point = []
        self.end_point = []  

        self.inittime = time.time()
        self.filee = open('/home/'+str(getpass.getuser())+'/versuch_b_'+str(int(self.inittime))+'.csv','w+')
        self.filee.write('zeit,distance_to_obstacle,aktivierungsfaktor,aktive_aufgaben\n')
        self.lastsavedtime = 0.0


    ### UPDATE CYCLE ###

    def update_world_model(self):

        for driver in self.drivers:
            driver.update()

        self.measurements = {}
        for link in self.links:
            link.update()
            self.measurements[link.symbol] = link.joint_value
        #print measurements

        for chain in self.chains.values():
            chain.update()

        for loop in self.loops.values():
            loop.update()

        # get vector components of feature frame ('ee_link' to 'target') ,target_to_eef
        self.target_ee_x = self.drivers[0].get_value('x1x')
        self.target_ee_y = self.drivers[0].get_value('x1y')
        self.target_ee_z = self.drivers[0].get_value('x1z')

        # calculate the length of vector 'ee_link' to 'obstacle'
        self.obstacle_distance = sqrt( self.drivers[1].get_value('x2x')**2 + self.drivers[1].get_value('x2y')**2 +self.drivers[1].get_value('x2z')**2)
        # calculate the length of vector 'ee_link' to 'target'
        self.target_distance = sqrt( self.target_ee_x**2 + self.target_ee_y**2 + self.target_ee_z**2)
        
        # if...elif... statement for movement of the target tf
        if self.target_distance <= 0.0002 and float(rospy.get_param('/target_z'))==self.start_point[2] :
            #sends [x,y,z ,a,b,c] for new target tf to the parameter server
            self.send_target(self.end_point)
        elif self.target_distance <= 0.0002 and float(rospy.get_param('/target_z'))==self.end_point[2]:
            # sends [x,y,z ,a,b,c] for new target tf to the parameter server
            self.send_target(self.start_point)

        # 
        if self.obstacle_distance <= 0.12:     

            # calculate the position of 'ee_link' in coordinates of 'base_link' frame ,base_to_target:
            if self.drivers[2].get_value('o1x') > 0:
                self.target_base_x = self.drivers[2].get_value('o1x') - self.target_ee_x
            else:
                self.target_base_x = self.drivers[2].get_value('o1x') + self.target_ee_x

            if self.drivers[2].get_value('o1y') > 0:
                self.target_base_y = self.drivers[2].get_value('o1y') - self.target_ee_y
            else:
                self.target_base_y = self.drivers[2].get_value('o1y') + self.target_ee_y

            if self.drivers[2].get_value('o1z') > 0:
                self.target_base_z = self.drivers[2].get_value('o1z') - self.target_ee_z
            else:
                self.target_base_z = self.drivers[2].get_value('o1z') + self.target_ee_z

            # activation function gives values between 0..1 for the distance 0.10 to 0.12
            self.active = -2500*self.obstacle_distance**2+600*self.obstacle_distance-35

            # avoid negative function values, when distance to obstacle is smaller 0.10
            if self.active < 0.0:
                self.active = 0.0

            # Syntax: atan2 (y,x) = [rad]   (from -> atan = y/x)e
            # rotation around x : linear line in yz plane
            roll = atan2((float(rospy.get_param('/target_z'))-self.target_base_z),(float(rospy.get_param('/target_y'))-self.target_base_y))
            rospy.set_param('obstacle_a', roll)

            # rotation around y : linear line in xz plane
            #pitch = atan2((float(rospy.get_param('/target_z'))-self.target_base_z),(float(rospy.get_param('/target_x'))-self.target_base_x))
            #rospy.set_param('obstacle_b', pitch)

            # rotation around z : linear line in xy plane
            yaw = atan2((float(rospy.get_param('/target_y'))-self.target_base_y),(float(rospy.get_param('/target_x'))-self.target_base_x))
            rospy.set_param('obstacle_c', yaw)

            # change tasks to task[0] and task[1], when not set already
            if self.tasks != [self.initial_tasks[0],self.initial_tasks[1]]:
                self.set_tasks([self.initial_tasks[0],self.initial_tasks[1]])
                self.wievieleaufgabeaktiv = 2
        else:
            # reset orientation of obstacel tf, when not in critical distance to obstacle
            rospy.set_param('obstacle_a', 0)
            rospy.set_param('obstacle_b', 0)
            rospy.set_param('obstacle_c', 0)

            # change tasks to task[1], here: reach target xyz abc
            # only do this, when not set. so no unnecessary call of 'build_tasks' is executed
            if self.tasks != [self.initial_tasks[1]]:
                self.set_tasks([self.initial_tasks[1]])
                self.wievieleaufgabeaktiv = 1

        ## save data for plots
        newtime = float(time.time())-self.inittime
        if self.lastsavedtime + 0.18 <= newtime: #5x per second
            self.filee.write( string.join([str(newtime),str(self.obstacle_distance),str(self.active),str(self.wievieleaufgabeaktiv)],';')+'\n')
            self.lastsavedtime = newtime

    ## send a pose to the parameter server,
    ## parameters are read by tf_target_braodcaster
    def send_target(self, pose):
        rospy.set_param('target_x', pose[0])
        rospy.set_param('target_y', pose[1])
        rospy.set_param('target_z', pose[2])
        rospy.set_param('target_a', pose[3])
        rospy.set_param('target_b', pose[4])
        rospy.set_param('target_c', pose[5])

    def set_initial_tasks(self,initial_tasks):
        with self.lock:

            self.initial_tasks = initial_tasks

# eof
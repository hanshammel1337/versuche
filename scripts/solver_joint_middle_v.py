#!/usr/bin/env python

## Python ##
##
import numpy as np
import time, string, getpass

## piTaSC ##
##
from pitasc_core.solver import Solver
import pitasc_core.kinematics as kin


class Solver_Jointmiddle(Solver):
    """ Computes the desired joint velocities dq_d based on the so called
        task priority strategy.
    """

    def __init__(self,scene):
        self.inittime = time.time()
        self.filee = open('/home/'+str(getpass.getuser())+'/versuch_a_'+str(scene.versuchsname)+'_'+str(int(self.inittime))+'.csv','w+')
        self.filee.write('zeit,joint_1_aktuell,untere_grenze,obere_grenze,mittelwert\n')
        self.lastsavedtime = 0.0

    def get_joint_vel(self, A_num, dy_d, scene):
        #q1min = 2.8407
        #q1max = 4.4112

        q1min = 1.26990367
        q1max = 2.8407
        q1mid = (q1min+q1max)/2
        """ Compute desired joint velocities based on priorities.

        Priorities are represented as follows:
        * There are priority levels that can be assigned to one or more
            constraints (which are imposed on the output equation).
        * The output equation, and therefore A_num and dy_d are ordered by
            the respective constraint priority, highest first.
        * The argument scene provides the current scene object with access
            to a lot of helpful data for the solver.
            Here the scene.priority_groups is used, which contains the number of
            constraints that share a priority level, highest level first.
        * As a consequence, the length of priority_groups indicates the number
            of priority levels while the sum over priority_groups equals the
            number of constraints.
        """
        newtime = float(time.time())-self.inittime
        if self.lastsavedtime + 0.18 <= newtime:
            self.filee.write( string.join([str(newtime),str(scene.measurements['joint_1']),str(q1min),str(q1max),str(q1mid)],';')+'\n')
            self.lastsavedtime = newtime

        ## Loop through tasks ##
        ##
        n = 0 # sum of all higher prioritized symbols
        m = 0 # number of symbols in current priority group
        for i in range(0, len(scene.priority_groups)):

            ## Current task ##
            ##
            m = scene.priority_groups[i]
            A_m = A_num[n:(n+m),:]
            y_m =  dy_d[n:(n+m)]


            ## Current task has highest priority ##
            ##
            Z = np.array([0.0,0.0,0.0, 0.0,0.0,0.0,0.0])
            Z[0] = -(scene.measurements['joint_1'] - q1mid) / ((q1max-q1min)**2)
            #print Z[0]
            A_m_inv = kin.sr_inverse(A_m) # singularity-robust inverse

            P1 = np.eye(A_m.shape[1]) - np.dot(A_m_inv, A_m)

            dq = np.dot(A_m_inv, y_m) + np.dot(P1,Z)


        #print scene.measurements['joint_1']
        return dq

# eof

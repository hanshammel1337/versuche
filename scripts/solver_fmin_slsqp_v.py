#!/usr/bin/env python

## Python ##
##
import numpy as np
import time, string, getpass
from scipy.optimize import fmin_slsqp

## piTaSC ##
##
from pitasc_core.solver import Solver


class Solver_FMIN_SLSQP(Solver):
    """ Computes the desired joint velocities dq_d
        Using numerical optimization with sequential least squares programming.
        Support for inequalities in cartesian and joint space.
    """

    def __init__(self,scene):
        self.x0 = np.array([0,0,0,0,0,0,0])
        self.inequaliy_list = []
        self.inittime = time.time()
        self.filee = open('/home/'+str(getpass.getuser())+'/versuch_c_'+str(scene.versuchsname)+'_'+str(int(self.inittime))+'.csv','w+')
        self.filee.write('zeit,joint_1,distance_to_obstacle,z_endeffektor,z_marker\n')
        self.lastsavedtime = 0.0


    def get_joint_vel(self, A_num, dy_d, scene):
        """ Compute desired joint velocities.
            Solves ||b-Ax||**2 with inequalities.
            Quadratic Problem, Quadratic Program
        """
        #start = time.time()
        newtime = float(time.time())-self.inittime
        if self.lastsavedtime + 0.18 <= newtime:
            self.filee.write( string.join([str(newtime),str(scene.measurements['joint_1']),str(scene.distance_to_obstacle),str(scene.endeffektor_z_soll),str(scene.endeffektor_z_ist)],';')+'\n')
            self.lastsavedtime = newtime
        
        ## set up a callable objective function to minimze
        ## least square ||b-Ax||**2 = b*b - 2*b*A + x*A*A*x
        ## here: ||dy_d - A_num * dq||**2
        A_trans = A_num.transpose()

        H = np.dot(A_trans,A_num)     #7x7

        c = dy_d.dot(A_num)

        def objective_function(x):
            return np.dot(x,np.dot(H,x))-np.dot(np.dot(2,c),x)

        ## call function for Sequential Least SQuares Programming
        dq = fmin_slsqp(func=objective_function,x0=self.x0,iprint=0,ieqcons=self.inequaliy_list,iter=2,acc=1e-8)

        ## set startpoint for next optimization to current dq values
        #self.x0 = dq

        #print 'freq:', round(1.0/(time.time() - start),2),'Hz'
        return dq

    ## Inequalities ##
    ##
    '''
        inequalities for fmin_slsqp: ieqcons (datatype = list)
        A list of functions of length n such
        that ieqcons[j](x,*args) >= 0.0 in a successfully optimized problem.
        The inequalities have to be computed as a function of the optimization vector x.
        In our case the x vector are the delta_q.

        a) define inequalities for joint values (joint space)
           e.g. actual_joint_value + delta_joint_value                   > joint_limit_min
                actual_joint_value + delta_joint_value - joint_limit_min > 0
        b) define inequalities in cartesian space
           build direct kinematic from the used robot in dependency of dq

        __Usage examples:__
        1) keep all joint 1 between limit_min and limit_max
        b1 = lambda x: np.array( (scene.measurements['joint_1'] + x[0]) - scene.links[0].limit_min )
        b2 = lambda x: np.array( scene.links[0].limit_max - (self.qm[0] + x[0]) )

        2) keep end effector z value between 0.1 and 0.5
        b3 = lambda x: direct_kinematic(x)[2] - 0.1
        b4 = lambda x: 0.5 - direct_kinematic(x)[2]

        solver.add_inequality([b1,b2,b3,b4])
    '''
    def add_inequality(self,funcs_add):
        for func_add in funcs_add:
            if (func_add is not None) and (func_add not in self.inequaliy_list):
                self.inequaliy_list.append(func_add)
                print "Adding inequality"
        

    def remove_inequality(self,funcs_remove):
        for func_remove in funcs_remove:
            if (func_remove is not None) and (func_remove in self.inequaliy_list):
                self.inequaliy_list.remove(func_remove)
                print "Removing inequality"

# eof

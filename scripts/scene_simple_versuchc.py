#!/usr/bin/env python

## Python ##
##
import numpy as np

import threading

from pitasc_core.scene import Scene
from pitasc_core.pitasc import PiTaSC


class Scene_Simple(Scene):
    
    def __init__(self):
        """ .

        """
        super(Scene_Simple, self).__init__()

        self.lock = threading.Lock()

        self.drivers = []
        self.links = []
        self.chains = {}

        self.tasks = [] # set via set_tasks()

        self.default_controller = None

        self.robot_drivers = []

        self.pitasc = PiTaSC()

        ##added fpr versuche
        self.distance_to_obstacle = 0.0
        self.endeffektor_z_soll = 0.0 #ee
        self.endeffektor_z_ist = 0.0  #marker
        self.versuchsname = ''


    ### UPDATE CYCLE ###

    def update_world_model(self):

        for driver in self.drivers:
            driver.update()

        self.measurements = {}
        for link in self.links:
            link.update()
            if link.symbol is not None:
                self.measurements[link.symbol] = link.joint_value
        #print self.measurements

        for chain in self.chains.values():
            chain.update()

        for loop in self.loops.values():
            loop.update()


    def update_pitasc(self):
        return self.pitasc.run(self)


    def update_robots(self, dq, q_symbols):
        for robot_driver in self.robot_drivers:
            robot_driver.set_joint_vel(dq, q_symbols)


    
    ### PARAMETERS ###

    ## Robot driver ##
    ##
    def add_robot_drivers(self, drivers):
        with self.lock:
            for driver in drivers:
                    if (driver is not None) and (driver not in self.robot_drivers):
                        self.robot_drivers.append(driver)
                        print "Adding robot driver 1"

                    if (driver is not None) and (driver not in self.drivers):
                        self.drivers.append(driver)
                        print "Adding robot driver 2"

    def remove_robot_drivers(self, drivers):
        with self.lock:
            for driver in drivers:
                    self.robot_drivers.remove(driver)
                    print "Removing robot driver 1"

                    self.drivers.remove(driver)
                    print "Removing robot driver 2"


    ## Driver ##
    ##
    def add_drivers(self, drivers):
        with self.lock:
            for driver in drivers:
                    if (driver is not None) and (driver not in self.drivers):
                        self.drivers.append(driver)
                        print "Adding driver"

    def remove_drivers(self, drivers):
        with self.lock:
            for driver in drivers:
                    self.drivers.remove(driver)
                    print "Removing driver"


    ## Links ##
    ##
    def add_links(self, links):
        with self.lock:
            for link in links:
                    if (link is not None) and (link not in self.links):
                        self.links.append(link)
                        print "Adding link:", link.symbol

    def remove_links(self, links):
        with self.lock:
            for link in links:
                    self.links.remove(link)
                    print "Removing link:", link.symbol


    ## Chains ##
    ##
    def add_chains(self, chains):
        with self.lock:
            ## If dict, take values
            if type(chains) == dict:
                chains = chains.values()

            ## Add all chains
            for chain in chains:
                if not chain in self.chains.values():
                    self.chains[chain.name] = chain
                    print "Adding chain:", chain.name

    def remove_chains(self, chains):
        with self.lock:
            for chain in chains.values():
                del self.chains[chain.name]
                print "Removing chain:", chain.name


    ## Loops ##
    ##
    def add_loops(self, loops):
        with self.lock:
            ## If dict, take values
            if type(loops) == dict:
                loops = loops.values()
                
            ## Add all loops
            for loop in loops:
                if not loop in self.loops.values():
                    self.loops[loop.name] = loop
                    print "Adding loop:", loop.name

    def remove_loops(self, loops):
        with self.lock:
            for loop in loops.values():
                del self.loops[loop.name]
                print "Removing loop:", loop.name


    ## Tasks ##
    ##
    def set_tasks(self, tasks):

        with self.lock:

            self.tasks = tasks

            self.build_tasks()



    ### BUILD ###

    def build_symbols(self):
        """ Rebuild, whenever the links change (system setup, controllable variables).

            Sets the parameters:
                q_symbols
                f_symbols

            Requires:
                links
        """
        print "1. Building symbols"

        ## Controllable Variables ##
        ##
        self.q_symbols = []
        self.f_symbols = []

        for link in self.links:

            ## Add symbol (depending on type) ##
            ##
            if(link.symbol_type == 'robot'):
                self.q_symbols += [link.symbol]
            elif(link.symbol_type == 'feature'):
                self.f_symbols += [link.symbol]
            

    def build_tasks(self):
        """ Rebuild, whenever the tasks change (task specification, output variables).

            Sets the parameters:
                y_symbols
                y_desired
                C_q
                C_f
                controllers

            Requires:
                q_symbols
                f_symbols
                tasks
                default_controller
        """
        print "2. Building tasks"

        if len(self.tasks) == 0:
            print "No tasks set"
            return

        ## Read out all output variables ##
        ##
        y_symbols = []
        y_desired = []
        for task in self.tasks:
            y_symbols.extend(task.symbols)
            y_desired.extend(task.desired)
        self.y_symbols = y_symbols
        self.y_desired = np.array(y_desired, dtype=np.float)

        print "q: ", self.q_symbols
        print "f: ", self.f_symbols
        print "y: ", self.y_symbols
        print "y_desired: ", self.y_desired


        ## Determine selection matrices ##
        ##
        ## Relate controllable and output variables

        self.C_q = np.zeros((len(self.y_symbols), len(self.q_symbols)))
        self.C_f = np.zeros((len(self.y_symbols), len(self.f_symbols)))

        for i, symbol in enumerate(self.y_symbols):

            if symbol in self.q_symbols:
                self.C_q[i,self.q_symbols.index(symbol)] = 1
            else:
                self.C_f[i,self.f_symbols.index(symbol)] = 1


        ## Controllers ##
        ##
        self.controllers = {} # dict {controller, symbol_indices}

        ## Specific controllers
        controlled_symbols = []
        for task in self.tasks:
            for (controller, symbols) in task.controllers:
                self.controllers[controller] = [y_symbols.index(symbol) for symbol in symbols]
                controlled_symbols.extend(symbols)

        ## Default controller for the rest
        self.controllers[self.default_controller] = [ y_symbols.index(symbol) for symbol in self.y_symbols if symbol not in controlled_symbols ]


        ## Priorities ##
        ##
        self.priority_groups = []
        for task in self.tasks:
            self.priority_groups.append(len(task.symbols))

# eof

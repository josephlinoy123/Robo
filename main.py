# -*- coding: utf-8 -*-
"""
Created on Mon Sep 24 12:01:49 2018

@author: jack
"""

from kinova_pb import *
from kinova_vr import *
from Rotations import *
          
experimentnum = 1  #Can Skip Early Experiments, ie skip to 3 to start with shapes
repeats = 20
""" See http://www.coppeliarobotics.com/
                                *****NOTE***** 
User path to VREP folder required, eg vrep_path = "V-REP_PRO_EDU_V3_6_2_Ubuntu18_04"
vrep.py, vrep.const and remoteAPI.so need to be added to the folder containing the main.py"""

vrep_path = "V-REP_PRO_EDU_V3_6_2_Ubuntu18_04"
         
if __name__ == '__main__':
    
    open_vrep(vrep_path)
    
    # Run all simulators for each of the experiments 1 - 10 
    for experimentnum in range(experimentnum,11):
        
        # Run the PyBullet Simulation for x Repeats 
        for iteration in range(repeats):
            simulate = PYBULLET(experimentnum)
            simulate.set_current_iteration(iteration)
            simulate.set_experiment(experimentnum)
            simulate.run_pybullet(experimentnum)
        print("PyBullet, Task %d is complete\n\n"%(experimentnum))
    
        # Run the VRep Simulation with Bullet283 physics engine for x Repeats 
        for iteration in range(repeats):
            simulate = VREP(experimentnum)
            simulate.set_current_iteration(iteration)
            simulate.set_experiment(experimentnum)
            simulate.Bullet283()
        print("Bullet283, Task %d is complete\n\n"%(experimentnum))
        
        # Run the VRep Simulation with Bullet278 physics engine for x Repeats 
        for iteration in range(repeats):
            simulate = VREP(experimentnum)
            simulate.set_current_iteration(iteration)
            simulate.set_experiment(experimentnum)
            simulate.Bullet278()
        print("Bullet278, Task %d is complete\n\n"%(experimentnum))
        # Run the VRep Simulation with ODE physics engine for x Repeats 
        for iteration in range(repeats):
            simulate = VREP(experimentnum)
            simulate.set_current_iteration(iteration)
            simulate.set_experiment(experimentnum)
            simulate.ODE()
        print("ODE, Task %d is complete\n\n"%(experimentnum))
        # Run the VRep Simulation with Vortex physics engine for x Repeats 
        for iteration in range(repeats):
            simulate = VREP(experimentnum)
            simulate.set_current_iteration(iteration)
            simulate.set_experiment(experimentnum)
            simulate.Vortex()
        print("Vortex, Task %d is complete\n\n"%(experimentnum))
        # Run the VRep Simulation with Newton physics engine for x Repeats 
        for iteration in range(repeats):
            simulate = VREP(experimentnum)
            simulate.set_current_iteration(iteration)
            simulate.set_experiment(experimentnum)
            simulate.Newton()
        print("Newton, Task %d is complete\n\n"%(experimentnum))
        print("Task %d is complete\n\n"%(experimentnum))

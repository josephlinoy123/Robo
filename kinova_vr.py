# -*- coding: utf-8 -*-
"""
Created on Mon May 28 14:01:53 2018

@author: jack
"""

import vrep
import sys
from PID import *
from Rotations import *
import csv
import math

from PID import *

simulator = "VRep"
physics_engine = "Bullet283"
experiment = "Single"
iteration = 0

class VREP(object):
    def __init__(self,experimentnumber):
        """Vrep Simulator, uses multiple engines. Uses .ttt file to generate arm and shape. 
        The majority of tuned parameters can be found in this class"""
        self._pid = [PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID()]
        self._linearVelocity = [0,0,0,0,0,0,0,0,0]
        self._theta = [0,0,0,0,0,0,0,0,0]
        self._positions =[]
        self._JointTorque =[]
        self._endeffector =[]
        self._objectState =[]
        self._compare=[]
        self._convertdeg2rad = 57.295779578552 
        self._num_steps = 0
        self._timestep = 0.05
        self._arm_joints = [0,0,0,0,0,0,0,0,0]
        self._measure_joint = [0]
        self._shape = 0
        
        self._simulator = "VRep"
        self._physics_engine = ""
        self._experiment = 0
        self._current_iteration = 0
        vrep.simxFinish(-1) # just in case, close all opened connections
        self._clientID=vrep.simxStart('127.0.0.1',19997,True,False,5000,5) # Connect to V-REP
        if experimentnumber == 1 or experimentnumber == 2:
            #standard Kinematics no cube
            if experimentnumber == 1:
                vrep.simxLoadScene(self._clientID, "kinova_description/urdf/m1n6s300.ttt", 1, vrep.simx_opmode_blocking)
            if experimentnumber == 2:
                vrep.simxLoadScene(self._clientID, "kinova_description/urdf/m1n6s300.ttt", 1, vrep.simx_opmode_blocking)     
        if experimentnumber == 3 or experimentnumber == 4:
            #Scene includes a respondable cube
            if experimentnumber == 3:
                vrep.simxLoadScene(self._clientID, "kinova_description/urdf/m1n6s300_cube.ttt", 1, vrep.simx_opmode_blocking)
                errorCode,self._shape = vrep.simxGetObjectHandle(self._clientID, 'Shape', vrep.simx_opmode_blocking)
                vrep.simxSetObjectFloatParameter(self._clientID,self._shape,vrep.sim_shapefloatparam_mass,0.07,vrep.simx_opmode_blocking)
                #vrep.simxSetObjectIntParameter()
            if experimentnumber == 4:
                vrep.simxLoadScene(self._clientID, "kinova_description/urdf/m1n6s300_cube.ttt", 1, vrep.simx_opmode_blocking)
                errorCode,self._shape = vrep.simxGetObjectHandle(self._clientID, 'Shape', vrep.simx_opmode_blocking)
                vrep.simxSetObjectFloatParameter(self._clientID,self._shape,vrep.sim_shapefloatparam_mass,0.46,vrep.simx_opmode_blocking)
        if experimentnumber == 5 or experimentnumber == 6:
            # Scene includes a respondablecylinder
            if experimentnumber == 5:
                #plastic
                vrep.simxLoadScene(self._clientID, "kinova_description/urdf/m1n6s300_cylinder.ttt", 1, vrep.simx_opmode_blocking)
                errorCode,self._shape = vrep.simxGetObjectHandle(self._clientID, 'Shape', vrep.simx_opmode_blocking)
                vrep.simxSetObjectFloatParameter(self._clientID,self._shape,vrep.sim_shapefloatparam_mass,0.12,vrep.simx_opmode_blocking)
            if experimentnumber == 6:
                #wood
                vrep.simxLoadScene(self._clientID, "kinova_description/urdf/m1n6s300_cylinder.ttt", 1, vrep.simx_opmode_blocking)
                errorCode,self._shape = vrep.simxGetObjectHandle(self._clientID, 'Shape', vrep.simx_opmode_blocking)
                vrep.simxSetObjectFloatParameter(self._clientID,self._shape,vrep.sim_shapefloatparam_mass,0.78,vrep.simx_opmode_blocking)
        if experimentnumber == 7 or experimentnumber == 8:
            #Scene includes a respondable cone, note dummy provided the tracking referrance
            if experimentnumber == 7:
                #plastic
                vrep.simxLoadScene(self._clientID, "kinova_description/urdf/m1n6s300_cone.ttt", 1, vrep.simx_opmode_blocking)
                errorCode,self._shape = vrep.simxGetObjectHandle(self._clientID, 'Shape', vrep.simx_opmode_blocking)
                vrep.simxSetObjectFloatParameter(self._clientID,self._shape,vrep.sim_shapefloatparam_mass,0.04,vrep.simx_opmode_blocking)
                errorCode,self._dummy = vrep.simxGetObjectHandle(self._clientID, 'Dummy', vrep.simx_opmode_blocking)
            if experimentnumber == 8:
                #wood
                vrep.simxLoadScene(self._clientID, "kinova_description/urdf/m1n6s300_cone.ttt", 1, vrep.simx_opmode_blocking)
                errorCode,self._shape = vrep.simxGetObjectHandle(self._clientID, 'Shape', vrep.simx_opmode_blocking)
                vrep.simxSetObjectFloatParameter(self._clientID,self._shape,vrep.sim_shapefloatparam_mass,0.17,vrep.simx_opmode_blocking)
                errorCode,self._dummy = vrep.simxGetObjectHandle(self._clientID, 'Dummy', vrep.simx_opmode_blocking)
        if experimentnumber == 9 or experimentnumber == 10:
            #Scene includes a respondable cuboid
            if experimentnumber == 9:
                #plastic
                vrep.simxLoadScene(self._clientID, "kinova_description/urdf/m1n6s300_cuboid.ttt", 1, vrep.simx_opmode_blocking)
                errorCode,self._shape = vrep.simxGetObjectHandle(self._clientID, 'Shape', vrep.simx_opmode_blocking)
                vrep.simxSetObjectFloatParameter(self._clientID,self._shape,vrep.sim_shapefloatparam_mass,0.14,vrep.simx_opmode_blocking)
            if experimentnumber == 10:
                #wood
                vrep.simxLoadScene(self._clientID, "kinova_description/urdf/m1n6s300_cuboid.ttt", 1, vrep.simx_opmode_blocking)
                errorCode,self._shape = vrep.simxGetObjectHandle(self._clientID, 'Shape', vrep.simx_opmode_blocking) 
                vrep.simxSetObjectFloatParameter(self._clientID,self._shape,vrep.sim_shapefloatparam_mass,0.85,vrep.simx_opmode_blocking)
        
        errorCode,self._arm_joints[0] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_1', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[1] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_2', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[2] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_3', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[3] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_4', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[4] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_5', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[5] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_6', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[6] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_finger_1', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[7] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_finger_2', vrep.simx_opmode_blocking)
        errorCode,self._arm_joints[8] = vrep.simxGetObjectHandle(self._clientID, 'm1n6s300_joint_finger_3', vrep.simx_opmode_blocking)
        errorCode,self._measure_joint[0] = vrep.simxGetObjectHandle(self._clientID,'gripper_mount_joint_7', vrep.simx_opmode_blocking)
        
        
    def set_current_iteration(self, iteration):
        self._current_iteration= iteration
        
    def set_experiment(self,experiment):
        self._experiment = experiment
        
    def set_num_steps(self):
        self._num_steps = simSteps(self._experiment,self._timestep) 
        
    def Bullet283(self):
        """Set Physics Engine Bullet283 """
        if self._clientID!=-1:
            self._physics_engine = "Bullet283"
            self.run_simulation(self._experiment)
        else:   #Else print error
            print ('Failed connecting to remote API server')
            sys.exit('Could not connect')
        
    def Bullet278(self):
        """Set Physics Engine Bullet278 """
        if self._clientID!=-1:
            vrep.simxSetIntegerParameter(self._clientID,vrep.sim_intparam_dynamic_engine,0,vrep.simx_opmode_blocking)
            self._physics_engine = "Bullet278"
            self.run_simulation(self._experiment)
        else:   #Else print error
            print ('Failed connecting to remote API server')
            sys.exit('Could not connect')
    
    def ODE(self):
        """Set Physics Engine ODE """
        if self._clientID!=-1:
            vrep.simxSetIntegerParameter(self._clientID,vrep.sim_intparam_dynamic_engine,1,vrep.simx_opmode_blocking)
            self._physics_engine = "ODE"
            self.run_simulation(self._experiment)
        else:   #Else print error# 
            print ('Failed connecting to remote API server')
            sys.exit('Could not connect')
        
    def Vortex(self):
        """Set Physics Engine Vortex """
        if self._clientID!=-1:
            vrep.simxSetIntegerParameter(self._clientID,vrep.sim_intparam_dynamic_engine,2,vrep.simx_opmode_blocking)
            self._physics_engine = "Vortex"
            self.run_simulation(self._experiment)
        else:   #Else print error
            print ('Failed connecting to remote API server')
            sys.exit('Could not connect')
    
    def Newton(self):
        """Set Physics Engine Newton """
        if self._clientID!=-1:
            vrep.simxSetIntegerParameter(self._clientID,vrep.sim_intparam_dynamic_engine,3,vrep.simx_opmode_blocking)
            self._physics_engine = "Newton"
            self.run_simulation(self._experiment)
        else:   #Else print error
            print ('Failed connecting to remote API server')
            sys.exit('Could not connect')
             
    def run_simulation(self,experimentnumber):
        """Runs this simulation for the number of steps while setting target thetas for joint movements via PID control"""
        self.set_num_steps()
                    
        vrep.simxSynchronous(self._clientID,True);
        vrep.simxStartSimulation(self._clientID,vrep.simx_opmode_oneshot);            
        
        for simStep in range (self._num_steps):
            
            self._pid = set_target_thetasVR(self._num_steps, self._pid,self._experiment,self._simulator,simStep,self._physics_engine)
            
            
            if simStep % 2 == 0:                   
                for joint in range(6):
                    #Control of the Arm through PID (see Rotations)
                    errorCode, self._theta[joint] = vrep.simxGetJointPosition(self._clientID,self._arm_joints[joint],vrep.simx_opmode_blocking)
                    self._linearVelocity[joint] = self._pid[joint].get_velocity(math.degrees(self._theta[joint]))/self._convertdeg2rad
                    vrep.simxSetJointTargetVelocity(self._clientID,self._arm_joints[joint],self._linearVelocity[joint],vrep.simx_opmode_oneshot)
                    
                #Saving arm stats
                arm_translation = [vrep.simxGetObjectPosition(self._clientID,self._measure_joint[0],-1, vrep.simx_opmode_blocking)[1][0],vrep.simxGetObjectPosition(self._clientID,self._measure_joint[0],-1, vrep.simx_opmode_blocking)[1][1],vrep.simxGetObjectPosition(self._clientID,self._measure_joint[0],-1, vrep.simx_opmode_blocking)[1][2]]
                arm_rotation = [vrep.simxGetObjectQuaternion(self._clientID,self._measure_joint[0],-1, vrep.simx_opmode_blocking)[1][3],vrep.simxGetObjectQuaternion(self._clientID,self._measure_joint[0],-1, vrep.simx_opmode_blocking)[1][0],vrep.simxGetObjectQuaternion(self._clientID,self._measure_joint[0],-1, vrep.simx_opmode_blocking)[1][1],vrep.simxGetObjectQuaternion(self._clientID,self._measure_joint[0],-1, vrep.simx_opmode_blocking)[1][2]]
                arm_torque = [vrep.simxGetJointForce(self._clientID,self._arm_joints[0],vrep.simx_opmode_blocking)[1],vrep.simxGetJointForce(self._clientID,self._arm_joints[1],vrep.simx_opmode_blocking)[1],vrep.simxGetJointForce(self._clientID,self._arm_joints[2],vrep.simx_opmode_blocking)[1],vrep.simxGetJointForce(self._clientID,self._arm_joints[3],vrep.simx_opmode_blocking)[1],vrep.simxGetJointForce(self._clientID,self._arm_joints[4],vrep.simx_opmode_blocking)[1],vrep.simxGetJointForce(self._clientID,self._arm_joints[5],vrep.simx_opmode_blocking)[1]]
                finger_position = [vrep.simxGetJointPosition(self._clientID,self._arm_joints[6],vrep.simx_opmode_blocking)[1],vrep.simxGetJointPosition(self._clientID,self._arm_joints[7],vrep.simx_opmode_blocking)[1],vrep.simxGetJointPosition(self._clientID,self._arm_joints[8],vrep.simx_opmode_blocking)[1]]
                force_torque = [vrep.simxReadForceSensor(self._clientID,self._measure_joint[0],vrep.simx_opmode_blocking)[2][0],vrep.simxReadForceSensor(self._clientID,self._measure_joint[0],vrep.simx_opmode_blocking)[2][1],vrep.simxReadForceSensor(self._clientID,self._measure_joint[0],vrep.simx_opmode_blocking)[2][2],vrep.simxReadForceSensor(self._clientID,self._measure_joint[0],vrep.simx_opmode_blocking)[3][0],vrep.simxReadForceSensor(self._clientID,self._measure_joint[0],vrep.simx_opmode_blocking)[3][1],vrep.simxReadForceSensor(self._clientID,self._measure_joint[0],vrep.simx_opmode_blocking)[3][2]]
                    
                if(experimentnumber == 1 or experimentnumber == 2):
                    #No shape info, only save Arm details
                    self._compare.append(arm_translation+arm_rotation+arm_torque+finger_position+force_torque)
                
                elif(experimentnumber == 7 or experimentnumber == 8):
                    #Cone has a dummy object attached to track its locations
                    object_translation = [vrep.simxGetObjectPosition(self._clientID,self._dummy,-1, vrep.simx_opmode_blocking)[1][0],vrep.simxGetObjectPosition(self._clientID,self._dummy,-1, vrep.simx_opmode_blocking)[1][1],vrep.simxGetObjectPosition(self._clientID,self._dummy,-1, vrep.simx_opmode_blocking)[1][2]]
                    object_rotation = [vrep.simxGetObjectQuaternion(self._clientID,self._dummy,-1, vrep.simx_opmode_blocking)[1][3],vrep.simxGetObjectQuaternion(self._clientID,self._dummy,-1, vrep.simx_opmode_blocking)[1][0],vrep.simxGetObjectQuaternion(self._clientID,self._dummy,-1, vrep.simx_opmode_blocking)[1][1],vrep.simxGetObjectQuaternion(self._clientID,self._dummy,-1, vrep.simx_opmode_blocking)[1][2]]
                    self._compare.append(arm_translation + arm_rotation + arm_torque + finger_position + force_torque + object_translation + object_rotation)
                else:
                    #All other objects are tracked without dummy
                    object_translation = [vrep.simxGetObjectPosition(self._clientID,self._shape,-1, vrep.simx_opmode_blocking)[1][0],vrep.simxGetObjectPosition(self._clientID,self._shape,-1, vrep.simx_opmode_blocking)[1][1],vrep.simxGetObjectPosition(self._clientID,self._shape,-1, vrep.simx_opmode_blocking)[1][2]]
                    object_rotation = [vrep.simxGetObjectQuaternion(self._clientID,self._shape,-1, vrep.simx_opmode_blocking)[1][3],vrep.simxGetObjectQuaternion(self._clientID,self._shape,-1, vrep.simx_opmode_blocking)[1][0],vrep.simxGetObjectQuaternion(self._clientID,self._shape,-1, vrep.simx_opmode_blocking)[1][1],vrep.simxGetObjectQuaternion(self._clientID,self._shape,-1, vrep.simx_opmode_blocking)[1][2]]
                    self._compare.append(arm_translation + arm_rotation + arm_torque + finger_position + force_torque + object_translation + object_rotation)
            #Stepping simulator
            vrep.simxSynchronousTrigger(self._clientID)
            vrep.simxGetPingTime(self._clientID)
        
        #Stop the simulation
        vrep.simxStopSimulation(self._clientID, vrep.simx_opmode_blocking)
        vrep.simxFinish(self._clientID)

        saveStats(self._experiment,self._current_iteration, self._physics_engine,self._simulator, self._JointTorque,self._endeffector,self._objectState,self._compare)

    

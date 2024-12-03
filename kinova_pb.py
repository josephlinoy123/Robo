# -*- coding: utf-8 -*-
"""
Created on Tue May  1 17:42:03 2018

@author: jack
"""
import time, os, math
from pybullet import *
import pybullet_data
from PID import *
from Rotations import *
import csv


class PYBULLET(object):
    def __init__(self, experimentnumber):
        """Pybullet Simulator.Uses .urdf to generate the arm and scene, uses .obj file to generate shapes. 
        The majority of tuned parameters can be found in this class"""
        self._pid = [PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID()]
        self._linearVelocity = [0,0,0,0,0,0,0,0,0]
        self._theta = [0,0,0,0,0,0,0,0,0]
        self._JointTorque =[]
        self._endeffector =[]
        self._objectState =[]
        self._compare=[]
        self._convertdeg2rad = 57.295779578552 
        self._num_steps = 0
        self._timestep = 1./240.
        
        self._simulator = "PyBullet"
        self._physics_engine = "Bullet"
        self._experiment = 0
        self._current_iteration = 0

        #Setup PyBullet
        self._physicsClient = connect(DIRECT)#or DIRECT for non-graphical version
        setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        setGravity(0,0,-9.81)
        resetDebugVisualizerCamera( cameraDistance=.4, cameraYaw=0, cameraPitch=5, cameraTargetPosition=[0.3,-0.5,.3])
        configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW,enable=0)
        configureDebugVisualizer(COV_ENABLE_SEGMENTATION_MARK_PREVIEW,enable=0)
        configureDebugVisualizer(COV_ENABLE_RGB_BUFFER_PREVIEW,enable=0)
        setPhysicsEngineParameter(fixedTimeStep = self._timestep)
        
        #Setup Floor
        self._planeId = loadURDF("plane.urdf")
                
        #Robot Arm
        self._kinovaStartPos = [0.0,0.0,0.055] # 
        self._kinova = loadURDF("kinova_description/urdf/m1n6s300.urdf",self._kinovaStartPos,useFixedBase=1)

        for jointNum in range(11): # allows the measurement of joint Torques
            enableJointForceTorqueSensor(self._kinova, jointNum)
        
        if experimentnumber == 1 or experimentnumber == 2:
            #standard Kinematics no Shape
            if experimentnumber == 1:
                ()                
            if experimentnumber == 2:
                ()
        if experimentnumber == 3 or experimentnumber == 4:
            #Scene includes a respondable cube
            if experimentnumber == 3:
                #plastic
                self.generate_cube(0.07)# mass parameter - plastic 
            if experimentnumber == 4:
                ()#wood
                self.generate_cube(0.46)# mass parameter - wood
        if experimentnumber == 5 or experimentnumber == 6:
            #Scene includes a respondable cylinder
            if experimentnumber == 5:
                ()#plastic
                self.generate_cylinder(0.12)  # mass parameter - plastic 
            if experimentnumber == 6:
                ()#wood
                self.generate_cylinder(0.78)  
        if experimentnumber == 7 or experimentnumber == 8:
            #Scene includes a respondable cone
            if experimentnumber == 7:
                ()#plastic
                self.generate_cone(0.04)  # mass parameter - plastic 
            if experimentnumber == 8:
                ()#wood
                self.generate_cone(0.17)  # mass parameter - wood
        if experimentnumber == 9 or experimentnumber == 10:
            #Scene includes a respondable cuboid
            if experimentnumber == 9:
                ()#plastic
                self.generate_cuboid(0.14)  # mass parameter - plastic 
            if experimentnumber == 10:
                ()#wood
                self.generate_cuboid(0.85)  # mass parameter - wood

    def generate_cube(self,mass):
        """generates a Cube shaped object based upon the supplied .obj file, places at start position and orientation, sets dynamic properties provided by gp_minimise
           The cube is set  with a mass for plastic in experiment 3 and wood for experiment 4.  
        """
        self._cube_mass = mass
        self._visualShapeId = -1
        self._cubeStartPos = [0.6,0.0,0.0375]
        self._cubeStartOrientation = getQuaternionFromEuler([0.0,0.0,0.0])
        createMultiBody(0,0)
        self._colCubeId = createCollisionShape(GEOM_MESH,fileName='kinova_description/Shapes/Cube.obj')
        self._objectUid = createMultiBody(self._cube_mass,self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
        
    def generate_cylinder(self,mass):
        """generates a cylinder shaped object based upon the supplied .obj file, places at start position and orientation, sets dynamic properties provided by gp_minimise
           The cylinder is set  with a mass for plastic in experiment 5 and wood for experiment 6.  
        """
        self._cylinder_mass = mass
        self._visualShapeId = -1
        self._cubeStartPos = [0.6,0.0,0.0375]
        self._cubeStartOrientation = getQuaternionFromEuler([1.571,0,0])
        createMultiBody(0,0)
        self._colCubeId = createCollisionShape(GEOM_MESH,fileName='kinova_description/Shapes/Cylinder.obj') #From obj mesh of cylinder 
        self._objectUid = createMultiBody(self._cylinder_mass,self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)        

    def generate_cuboid(self,mass):
        """generates a cuboid shaped object based upon the supplied .obj file, places at start position and orientation, sets dynamic properties provided by gp_minimise
           The cuboid is set  with a mass for plastic in experiment 9 and wood for experiment 10.  
        """ 
        self._cube_mass = mass
        self._visualShapeId = -1
        self._cubeStartPos = [0.6,0.0,0.075]
        self._cubeStartOrientation = getQuaternionFromEuler([0.0,0.0,0.0]) #radians
        createMultiBody(0,0)
        self._colCubeId = createCollisionShape(GEOM_MESH,fileName='kinova_description/Shapes/Cuboid.obj')
        self._objectUid = createMultiBody(self._cube_mass,self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
                
    def generate_cone(self,mass):
        """generates a cone shaped object based upon the supplied .obj file, places at start position and orientation, sets dynamic properties provided by gp_minimise
           The cone is set  with a mass for plastic in experiment 7 and wood for experiment 8.  
        """
        self._cube_mass = mass
        self._visualShapeId = -1
        self._cubeStartPos = [0.6,0.00,0.0375]
        self._cubeStartOrientation = getQuaternionFromEuler([0.0,1.92957,0.0])
        createMultiBody(0,0)
        #createvisualShape... 
        self._colCubeId = createCollisionShape(GEOM_MESH,fileName='kinova_description/Shapes/Cone.obj')

        self._objectUid = createMultiBody(self._cube_mass,self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
                
    def set_current_iteration(self, iteration):
        self._current_iteration= iteration
        
    def set_experiment(self,experimentnumber):
        self._experiment = experimentnumber
        
    def set_num_steps(self):
        self._num_steps = simSteps(self._experiment,self._timestep)
        
    def lock_joints(self):
        for joint in range (getNumJoints(self._kinova)):
          setJointMotorControl2(self._kinova,jointIndex=joint,controlMode=POSITION_CONTROL, targetPosition=0, force=2000)
    
    def run_pybullet(self,experimentnumber):
        """Runs this simulation for the number of steps while setting target thetas for joint movements via PID control"""
        self.set_num_steps()
               
        self.lock_joints()
        for simStep in range(self._num_steps):
            
            self._pid = set_target_thetas(self._num_steps, self._pid,self._experiment,self._simulator,simStep,self._physics_engine)
            
            
            if simStep % 24 == 0:
                for jointNum in range(9):
                    self._theta[jointNum] = getJointState(self._kinova, jointNum)[0]
                    self._linearVelocity[jointNum] = self._pid[jointNum].get_velocity(math.degrees(self._theta[jointNum]))/self._convertdeg2rad
                    setJointMotorControl2(bodyIndex=self._kinova,jointIndex=jointNum,controlMode=VELOCITY_CONTROL,targetVelocity=self._linearVelocity[jointNum],force=2000)
                
                    arm_translation = [getLinkState(self._kinova,10,1)[0][0],getLinkState(self._kinova,10,1)[0][1],getLinkState(self._kinova,10,1)[0][2]]
                    arm_rotation = [getLinkState(self._kinova,10,1)[1][3],getLinkState(self._kinova,10,1)[1][0],getLinkState(self._kinova,10,1)[1][1],getLinkState(self._kinova,10,1)[1][2]]
                    arm_torque = [getJointState(self._kinova,2)[3],getJointState(self._kinova,3)[3],getJointState(self._kinova,4)[3],getJointState(self._kinova,5)[3],getJointState(self._kinova,6)[3],getJointState(self._kinova,7)[3]]
                    finger_position = [getJointState(self._kinova,15)[0],getJointState(self._kinova,16)[0],getJointState(self._kinova,17)[0]]
                    force_torque = [getJointState(self._kinova,9)[2][0],getJointState(self._kinova,9)[2][1],getJointState(self._kinova,9)[2][2],getJointState(self._kinova,9)[2][3],getJointState(self._kinova,9)[2][4],getJointState(self._kinova,9)[2][5]]
                
                if(experimentnumber == 1 or experimentnumber == 2):
                    self._compare.append(arm_translation+arm_rotation+arm_torque+finger_position+force_torque)
                else:
                    object_translation = [getBasePositionAndOrientation(self._objectUid)[0][0],getBasePositionAndOrientation(self._objectUid)[0][1],getBasePositionAndOrientation(self._objectUid)[0][2]]
                    object_rotation = [getBasePositionAndOrientation(self._objectUid)[1][3],getBasePositionAndOrientation(self._objectUid)[1][0],getBasePositionAndOrientation(self._objectUid)[1][1],getBasePositionAndOrientation(self._objectUid)[1][2]]
                    self._compare.append(arm_translation + arm_rotation + arm_torque + finger_position + force_torque + object_translation + object_rotation)
            
            stepSimulation()
            time.sleep(self._timestep)
    
        disconnect()

        saveStats(self._experiment,self._current_iteration, self._physics_engine,self._simulator, self._JointTorque,self._endeffector,self._objectState,self._compare)

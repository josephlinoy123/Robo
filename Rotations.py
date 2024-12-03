# -*- coding: utf-8 -*-
"""
Created on Mon Sep 24 13:55:07 2018

@author: jack
"""
from PID import *
import subprocess
import time
import csv
import os

def simSteps(experiment,timestep):      
    #seconds = 10
    if experiment == 1:
        seconds = 10
    elif experiment == 2:
        seconds = 14
    elif experiment == 3 or experiment == 4:
        seconds = 13
    elif experiment == 5 or experiment == 6:
        seconds = 79
    elif experiment == 7 or experiment == 8:
        seconds = 69
    elif experiment == 9 or experiment == 10:
        seconds = 9
       
    return int(seconds/timestep)
        
def set_target_thetas(num_steps, pid, experiment,simulator, simStep,physics_engine):
    """Operates for PyBullet - Sets the target thetas for each of the joints for the relevent time segments, 
    modify these to match your desired arm travel paths """
    if (experiment == 1): # 10 Seconds total time 
        #TASK 1: 
        if (simStep > 0) and (simStep < 720): # start from 0 
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(90-180)
            pid[2].set_target_theta(180-180)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(0)
            pid[5].set_target_theta(0)
        elif (simStep >=720) and (simStep<1440):  
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(180-180)
            pid[2].set_target_theta(270-180)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(0)
            pid[5].set_target_theta(0)
        elif (simStep >=1440): 
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(90-180)
            pid[2].set_target_theta(180-180)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(90)
            pid[5].set_target_theta(0)
                
    elif (experiment == 2): # 14 seconds @ 1/240 timestep 
        #TASK2:12:10 am - 12:1011/7/19Temp = 21.6 Humidity = 50%
        if (simStep > 0) and (simStep < 720): # start from 0 
            pid[0].set_target_theta(-2)
            pid[1].set_target_theta(177-180)
            pid[2].set_target_theta(172-180)
            pid[3].set_target_theta(10)
            pid[4].set_target_theta(-2)
            pid[5].set_target_theta(0)
        elif (simStep >=720) and (simStep < 1200) :
            pid[0].set_target_theta(47)
            pid[1].set_target_theta(160-180)
            pid[2].set_target_theta(214-180)
            pid[3].set_target_theta(8)
            pid[4].set_target_theta(-19)
            pid[5].set_target_theta(0)
        elif (simStep >=1200) and (simStep < 1680) :
            pid[0].set_target_theta(46)
            pid[1].set_target_theta(157-180)
            pid[2].set_target_theta(250-180)
            pid[3].set_target_theta(13)
            pid[4].set_target_theta(-42)
            pid[5].set_target_theta(0)
        elif (simStep >=1680) and (simStep < 2160) :
            pid[0].set_target_theta(19)
            pid[1].set_target_theta(143-180)
            pid[2].set_target_theta(263-180)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(-63)
            pid[5].set_target_theta(0)
        elif (simStep >=2160) and (simStep < 2640) :
            pid[0].set_target_theta(-23)
            pid[1].set_target_theta(154-180)
            pid[2].set_target_theta(284-180)
            pid[3].set_target_theta(-88)
            pid[4].set_target_theta(-47)
            pid[5].set_target_theta(0)
        elif (simStep >=2640):
            pid[0].set_target_theta(-32)
            pid[1].set_target_theta(160-180)
            pid[2].set_target_theta(214-180)
            pid[3].set_target_theta(2)
            pid[4].set_target_theta(-21)
            pid[5].set_target_theta(0)

    elif (experiment == 3) or (experiment == 4): # Task 6  total 13 seconds @ 1/240 timestep 
        #TASk 6:
        if (simStep > 0) and (simStep < 720): # start from 0 
            pid[0].set_target_theta(-3)
            pid[1].set_target_theta(150-180)
            pid[2].set_target_theta(281-180)
            pid[3].set_target_theta(97)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(-11)
        elif (simStep >=720)and (simStep < 1200):
            pid[0].set_target_theta(-3)
            pid[1].set_target_theta(123-180)
            pid[2].set_target_theta(253-180)
            pid[3].set_target_theta(101)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(-12)
        elif (simStep >=1200) and (simStep < 1680):
            pid[0].set_target_theta(-3)
            pid[1].set_target_theta(114-180)
            pid[2].set_target_theta(224-180)
            pid[3].set_target_theta(101)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(-12)
        elif (simStep >=1680) and (simStep < 2160):
            pid[0].set_target_theta(15)
            pid[1].set_target_theta(120-180)
            pid[2].set_target_theta(229-180)
            pid[3].set_target_theta(107)
            pid[4].set_target_theta(-3)
            pid[5].set_target_theta(-96)
        elif (simStep >=2160) and (simStep < 2640):
            pid[0].set_target_theta(15)
            pid[1].set_target_theta(96-180)
            pid[2].set_target_theta(208-180)
            pid[3].set_target_theta(107)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(-96)
        elif (simStep >=2640):
            pid[0].set_target_theta(-20)
            pid[1].set_target_theta(96-180)
            pid[2].set_target_theta(208-180)
            pid[3].set_target_theta(107)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(-96)

    elif (experiment == 5) or (experiment == 6):
        #TASK 7: 79 Seconds
        if simStep > 0 and simStep < 720: # start from 0 
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(169-180)
            pid[2].set_target_theta(285-180)
            pid[3].set_target_theta(87)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(1)
        elif simStep >=720 and simStep < 1440:
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(140-180)
            pid[2].set_target_theta(274-180)
            pid[3].set_target_theta(87)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(1)
        elif simStep >=1440 and simStep < 2160:
            pid[0].set_target_theta(2)
            pid[1].set_target_theta(123-180)
            pid[2].set_target_theta(250-180)
            pid[3].set_target_theta(87)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(1)
        elif simStep >=2160 and simStep < 4560:
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(123-180)
            pid[2].set_target_theta(243-180)
            pid[3].set_target_theta(87)
            pid[4].set_target_theta(-5)
            pid[5].set_target_theta(1)
        elif simStep >=4560:
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(123-180)
            pid[2].set_target_theta(243-180)
            pid[3].set_target_theta(87)
            pid[4].set_target_theta(-5)
            pid[5].set_target_theta(1)

    elif (experiment == 7) or (experiment == 8):
        if (simStep > 0)and(simStep < 720): # start from 0 
            #TASK 9: 69 Seconds 
            pid[0].set_target_theta(-11)
            pid[1].set_target_theta(114-180)
            pid[2].set_target_theta(185-180)
            pid[3].set_target_theta(11)
            pid[4].set_target_theta(-58)
            pid[5].set_target_theta(100)
        elif(simStep >=720) and (simStep < 1440):
            pid[0].set_target_theta(2)
            pid[1].set_target_theta(92-180)
            pid[2].set_target_theta(185-180)
            pid[3].set_target_theta(11)
            pid[4].set_target_theta(-49)
            pid[5].set_target_theta(100)
        elif(simStep >=1440) and (simStep < 2160):
            pid[0].set_target_theta(9)
            pid[1].set_target_theta(92-180)
            pid[2].set_target_theta(188-180)
            pid[3].set_target_theta(11)
            pid[4].set_target_theta(-49)
            pid[5].set_target_theta(100)
        elif(simStep >=2160):
            pid[0].set_target_theta(9)
            pid[1].set_target_theta(92-180)
            pid[2].set_target_theta(188-180)
            pid[3].set_target_theta(11)
            pid[4].set_target_theta(-49)
            pid[5].set_target_theta(100)
            
    elif(experiment == 9) or (experiment == 10):
        #TASK 10: 9 seconds 
        if (simStep > 0) and (simStep < 720): # start from 0 
            pid[0].set_target_theta(-5)
            pid[1].set_target_theta(166-180)
            pid[2].set_target_theta(290-180)
            pid[3].set_target_theta(91)
            pid[4].set_target_theta(0)
            pid[5].set_target_theta(0)
        elif(simStep >=720) and (simStep < 1440):
            pid[0].set_target_theta(-2)
            pid[1].set_target_theta(140-180)
            pid[2].set_target_theta(265-180)
            pid[3].set_target_theta(91)
            pid[4].set_target_theta(0)
            pid[5].set_target_theta(0)
        elif(simStep >=1440) and (simStep < 2160):
            pid[0].set_target_theta(-2)
            pid[1].set_target_theta(130-180)
            pid[2].set_target_theta(200-180)
            pid[3].set_target_theta(91)
            pid[4].set_target_theta(0)
            pid[5].set_target_theta(0)
    

    if simulator == "PyBullet":
        pid = pid[-2:] + pid[0:-2]

    return pid
    
def open_vrep(vrep_path):
    """Opens Vrep Window"""
    terminal_string = "cd " + vrep_path + " && ./vrep.sh -h"
    proc = subprocess.Popen([terminal_string], shell=True)
    time.sleep(8) # takes a moment to start
    proc.terminate()

def saveStats(experiment, iteration, physics_engine, simulator, joint_torque, end_effector,object_state,compare):
    """Saves statistics to destination .csv files """
    if iteration < 9:
        fileStringComp = 'Results/task%d/%s_%s/'%(experiment,simulator, physics_engine) + '0%d.csv'%(iteration+1)
    else:
        fileStringComp = 'Results/task%d/%s_%s/'%(experiment,simulator, physics_engine) + '%d.csv'%(iteration+1)

    os.makedirs(os.path.dirname(fileStringComp), exist_ok=True)
    
    with open(fileStringComp, 'w+', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        for xyz in compare:
            writer.writerow(xyz)
            
def set_target_thetasVR(num_steps, pid, experiment,simulator, simStep,physics_engine):
    """Operates for VRep and associated engines - Sets the target thetas for each of the joints for the relevent time segments, 
    modify these to match your desired arm travel paths """
    
    if (experiment == 1): # 10 Seconds total time 
        #Everything is at 20Hz so range(60) means do this for 3 seconds     
        #TASK 1: 
        if (simStep >= 0) and (simStep < 60): # start from 0 
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(90-180)
            pid[2].set_target_theta(180-180)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(0)
            pid[5].set_target_theta(0)
        elif (simStep >=60 ) and (simStep<120):  
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(180-180)
            pid[2].set_target_theta(270-180)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(0)
            pid[5].set_target_theta(0)
        elif (simStep >=120): 
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(90-180)
            pid[2].set_target_theta(180-180)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(90)
            pid[5].set_target_theta(0)
                
    elif (experiment == 2): # 14 seconds @ 1/240 timestep 
        #TASK2:12:10 am - 12:1011/7/19Temp = 21.6 Humidity = 50%
        if (simStep >= 0) and (simStep < 60): # start from 0 
            pid[0].set_target_theta(-2)
            pid[1].set_target_theta(177-180)
            pid[2].set_target_theta(172-180)
            pid[3].set_target_theta(10)
            pid[4].set_target_theta(-2)
            pid[5].set_target_theta(0)
        elif (simStep >=60) and (simStep < 100) :
            pid[0].set_target_theta(47)
            pid[1].set_target_theta(160-180)
            pid[2].set_target_theta(214-180)
            pid[3].set_target_theta(8)
            pid[4].set_target_theta(-19)
            pid[5].set_target_theta(0)
        elif (simStep >=100) and (simStep < 140) :
            pid[0].set_target_theta(46)
            pid[1].set_target_theta(157-180)
            pid[2].set_target_theta(250-180)
            pid[3].set_target_theta(13)
            pid[4].set_target_theta(-42)
            pid[5].set_target_theta(0)
        elif (simStep >=140) and (simStep < 180) :
            pid[0].set_target_theta(19)
            pid[1].set_target_theta(143-180)
            pid[2].set_target_theta(263-180)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(-63)
            pid[5].set_target_theta(0)
        elif (simStep >=180) and (simStep < 220) :
            pid[0].set_target_theta(-23)
            pid[1].set_target_theta(154-180)
            pid[2].set_target_theta(284-180)
            pid[3].set_target_theta(-88)
            pid[4].set_target_theta(-47)
            pid[5].set_target_theta(0)
        elif (simStep >=220):
            pid[0].set_target_theta(-32)
            pid[1].set_target_theta(160-180)
            pid[2].set_target_theta(214-180)
            pid[3].set_target_theta(2)
            pid[4].set_target_theta(-21)
            pid[5].set_target_theta(0)

    elif (experiment == 3) or (experiment == 4): # Task 6  total 13 seconds @ 1/240 timestep 
        #TASk 6:
        if (simStep >= 0) and (simStep < 60): # start from 0 
            pid[0].set_target_theta(-3)
            pid[1].set_target_theta(150-180)
            pid[2].set_target_theta(281-180)
            pid[3].set_target_theta(97)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(-11)
        elif (simStep >=60)and (simStep < 100):
            pid[0].set_target_theta(-3)
            pid[1].set_target_theta(123-180)
            pid[2].set_target_theta(253-180)
            pid[3].set_target_theta(101)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(-12)
        elif (simStep >=100) and (simStep < 140):
            pid[0].set_target_theta(-3)
            pid[1].set_target_theta(114-180)
            pid[2].set_target_theta(224-180)
            pid[3].set_target_theta(101)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(-12)
        elif (simStep >=140) and (simStep < 180):
            pid[0].set_target_theta(15)
            pid[1].set_target_theta(120-180)
            pid[2].set_target_theta(229-180)
            pid[3].set_target_theta(107)
            pid[4].set_target_theta(-3)
            pid[5].set_target_theta(-96)
        elif (simStep >=180) and (simStep < 220):
            pid[0].set_target_theta(15)
            pid[1].set_target_theta(96-180)
            pid[2].set_target_theta(208-180)
            pid[3].set_target_theta(107)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(-96)
        elif (simStep >=220):
            pid[0].set_target_theta(-20)
            pid[1].set_target_theta(96-180)
            pid[2].set_target_theta(208-180)
            pid[3].set_target_theta(107)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(-96)

    elif (experiment == 5) or (experiment == 6):
        #TASK 7: 79 Seconds
        if simStep >= 0 and simStep < 60: # start from 0 
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(169-180)
            pid[2].set_target_theta(285-180)
            pid[3].set_target_theta(87)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(1)
        elif simStep >=60 and simStep < 120:
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(140-180)
            pid[2].set_target_theta(274-180)
            pid[3].set_target_theta(87)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(1)
        elif simStep >=120 and simStep < 180:
            pid[0].set_target_theta(2)
            pid[1].set_target_theta(123-180)
            pid[2].set_target_theta(250-180)
            pid[3].set_target_theta(87)
            pid[4].set_target_theta(-4)
            pid[5].set_target_theta(1)
        elif simStep >=180 and simStep < 380:
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(123-180)
            pid[2].set_target_theta(243-180)
            pid[3].set_target_theta(87)
            pid[4].set_target_theta(-5)
            pid[5].set_target_theta(1)
        elif simStep >=380:
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(123-180)
            pid[2].set_target_theta(243-180)
            pid[3].set_target_theta(87)
            pid[4].set_target_theta(-5)
            pid[5].set_target_theta(1)

    elif (experiment == 7) or (experiment == 8):
        if (simStep >= 0)and(simStep < 60): # start from 0 
            #TASK 9: 69 Seconds 
            pid[0].set_target_theta(-11)
            pid[1].set_target_theta(114-180)
            pid[2].set_target_theta(185-180)
            pid[3].set_target_theta(11)
            pid[4].set_target_theta(-58)
            pid[5].set_target_theta(100)
        elif(simStep >=60) and (simStep < 120):
            pid[0].set_target_theta(2)
            pid[1].set_target_theta(92-180)
            pid[2].set_target_theta(185-180)
            pid[3].set_target_theta(11)
            pid[4].set_target_theta(-49)
            pid[5].set_target_theta(100)
        elif(simStep >=120) and (simStep < 180):
            pid[0].set_target_theta(9)
            pid[1].set_target_theta(92-180)
            pid[2].set_target_theta(188-180)
            pid[3].set_target_theta(11)
            pid[4].set_target_theta(-49)
            pid[5].set_target_theta(100)
        elif(simStep >=180):
            pid[0].set_target_theta(9)
            pid[1].set_target_theta(92-180)
            pid[2].set_target_theta(188-180)
            pid[3].set_target_theta(11)
            pid[4].set_target_theta(-49)
            pid[5].set_target_theta(100)
            
    elif(experiment == 9) or (experiment == 10):
        #TASK 10: 9 seconds 
        if (simStep >= 0) and (simStep < 60): # start from 0 
            pid[0].set_target_theta(-5)
            pid[1].set_target_theta(166-180)
            pid[2].set_target_theta(290-180)
            pid[3].set_target_theta(91)
            pid[4].set_target_theta(0)
            pid[5].set_target_theta(0)
        elif(simStep >=60) and (simStep < 120):
            pid[0].set_target_theta(-2)
            pid[1].set_target_theta(140-180)
            pid[2].set_target_theta(265-180)
            pid[3].set_target_theta(91)
            pid[4].set_target_theta(0)
            pid[5].set_target_theta(0)
        elif(simStep >=120) and (simStep < 180):
            pid[0].set_target_theta(-2)
            pid[1].set_target_theta(130-180)
            pid[2].set_target_theta(200-180)
            pid[3].set_target_theta(91)
            pid[4].set_target_theta(0)
            pid[5].set_target_theta(0)


    return pid
# -*- coding: utf-8 -*-
"""
Benchmarking Simulated Robotic Manipulation through a Real World Dataset

Script for calculating metrics that compare simulations to real-world dataset.

Contact: Jack Collins
Email: jack.collins@data61.csiro.au
"""

#Import required modules
import sys, getopt, math, csv
import numpy as np
from scipy.spatial import distance
from scipy import linalg
from pathlib import Path

frequency = 0.1

#Import 20 repeats of simulation data
def readSim(simulated_task_location):
    try:
        #20 repeats with naming starting at 01
        for simRepeat in range(1,21):
            if simRepeat < 10:
                file = simulated_task_location[0:-6] + '0' + str(simRepeat) + ".csv"
            else:
                file = simulated_task_location[0:-6] + str(simRepeat) + ".csv"
            tmp = np.genfromtxt(Path(file), comments="#", delimiter=",") #Open file and read into numpy array
            # Create 3D numpy array of the 20 repeats
            if simRepeat == 1:
                np_sim_results = tmp
            else:
                np_sim_results = np.dstack((np_sim_results,tmp))
        #Find mean of all data except object positions
        if np_sim_results.shape[1] > 22:
            mean_sim_results = np.mean(np_sim_results[:,0:22,:], axis=2)

            for simRepeat in range(20):
                mean_sim_results = np.append(mean_sim_results,np_sim_results[:,22:29,simRepeat],axis=1)
        else:
            mean_sim_results = np.mean(np_sim_results, axis=2)

        return mean_sim_results
    except:
        print("""Unable to open simulation data files, check you are following the
        specified file format and the location contains 20 repeats of the simulation
        with enumerated file names\n""")
        sys.exit(2)

#Import the mean Ground Truth data
def readGroundtruth(groundtruth_location,meanSim):
    #Skip header and column labels when reading data
    np_data_results = np.genfromtxt(Path(groundtruth_location), comments="#",skip_header=3, delimiter=",") #Open file and read into numpy array
    np_data_results = np_data_results[2:,:]
    #Make dataset the same length as sim data
    if np_data_results.shape[0] > meanSim.shape[0]:
        np_data_results = np_data_results[0:meanSim.shape[0],:]
    elif np_data_results.shape[0] < meanSim.shape[0]:
        difference  = meanSim.shape[0] - np_data_results.shape[0]
        for row in range(difference):
            np_data_results = np.vstack((np_data_results,np_data_results[-1,:]))
    #Check if last row of data has zeros in it
    rows = 1
    row_zero = True
    while row_zero == True:
        row_zero = False
        for col in range(np_data_results.shape[1]):
            if np_data_results[-rows,col] == 0 and col != 13 and col != 14 and col != 15:
                row_zero = True
                rows+=1
                break
    #If zeros then replace with the last rows without zeros
    for row in range(1,rows):
        np_data_results[-(row),:] = np_data_results[-rows,:]

    return np_data_results

#Conversion from Quarternion to Rotation Matrix
def RfromQuat(w,x,y,z):
    rotMatrix = np.zeros((3,3),np.float64)
    rotMatrix[0][0] = 1 - 2*(y**2) - 2*(z**2)
    rotMatrix[0][1] = 2*x*y - 2*z*w
    rotMatrix[0][2] = 2*x*z + 2*y*w
    rotMatrix[1][0] = 2*x*y + 2*z*w
    rotMatrix[1][1] = 1 - 2*(x**2) - 2*(z**2)
    rotMatrix[1][2] = 2*y*z - 2*x*w
    rotMatrix[2][0] = 2*x*z - 2*y*w
    rotMatrix[2][1] = 2*y*z + 2*x*w
    rotMatrix[2][2] = 1 - 2*(x**2) - 2*(y**2)
    return rotMatrix

#Conversion from Quarternion to Euler angles (roll, pitch yaw)
def EulerfromQuat(w,x,y,z):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return np.array((X,Y,Z))

#Calculates the velocity given two 3D positions
def velocity(a,b):
    global frequency
    dist_sim = distance.euclidean(a, b)
    return dist_sim/frequency

#Calculates acceleration given three 3D positions
def acceleration(a,b,c):
    global frequency
    past_velocity = velocity(a,b)
    current_velocity = velocity(b,c)
    return (past_velocity-current_velocity)/frequency

#Calcultes the Euclidean Error between simulation and dataset
def euclideanError(meanSim,meanGroundtruth):
    error = 0
    for step in range(meanSim.shape[0]):
        step_error = distance.euclidean(meanSim[step,0:3], meanGroundtruth[step,0:3])
        error += step_error
    return error/meanSim.shape[0]

#Calcultes the Rotational Error between simulation and dataset
#Uses Inner Product of Unit Quarternions from D. Q. Huynh, “Metrics for 3D rotations: Comparison and analysis,” J. Math. Imaging Vis., vol. 35, no. 2, pp. 155–164, 2009.
def rotationError(meanSim,meanGroundtruth):
    error = 0
    for step in range(meanSim.shape[0]):
        simR = np.array([meanSim[step,3],meanSim[step,4],meanSim[step,5],meanSim[step,6]])
        groundtruthR = np.array([meanGroundtruth[step,3],meanGroundtruth[step,4],meanGroundtruth[step,5],meanGroundtruth[step,6]])
        step_error = np.arccos(np.abs(np.dot(simR,groundtruthR)))
        error += step_error
    return error/meanSim.shape[0]

#Calcultes the Pose Error (Combines both translation and rotation) between simulation and dataset
#Calculated using F. C. Park, “Distance Metrics on the Rigid-Body Motions with Applications to Mechanism Design,” J. Mech. Des., vol. 117, no. 1, pp. 48–54, Mar. 1995.
def poseError(meanSim,meanGroundtruth):
    error = 0
    scale = 37 #Rotation has a volume of 8*Pi^2 therefore the workspace volume of the arm should be scaled accordingly. The workspace of the arm is 0.8m
    for step in range(meanSim.shape[0]):
        sim_rotation = RfromQuat(meanSim[step,3],meanSim[step,4],meanSim[step,5],meanSim[step,6])
        dataset_rotation = RfromQuat(meanGroundtruth[step,3],meanGroundtruth[step,4],meanGroundtruth[step,5],meanGroundtruth[step,6])
        rotation_error = (np.linalg.norm(linalg.logm(np.linalg.inv(dataset_rotation)*sim_rotation)))**2
        sim_vector = [meanSim[step,0],meanSim[step,1],meanSim[step,2]]
        dataset_vector = [meanGroundtruth[step,0],meanGroundtruth[step,1],meanGroundtruth[step,2]]
        translation_error = (np.linalg.norm(np.subtract(sim_vector,dataset_vector)))**2
        scaled_translation = scale*translation_error
        error+= rotation_error+scaled_translation
    return error/meanSim.shape[0]

#Calcultes the absolute velocity error between simulation and dataset as well as max and min velocities of simulation
def velocityError(meanSim,meanGroundtruth):
    minVelocitySim = 1000.0 # start at large number
    maxVelocitySim = 0.0  # start at small number
    error = 0

    for step in range(meanSim.shape[0]-1):
        velocity_sim = velocity(meanSim[step+1,0:3],meanSim[step,0:3])
        velocity_dataset = velocity(meanGroundtruth[step+1,0:3], meanGroundtruth[step,0:3])

        if (velocity_sim > maxVelocitySim): maxVelocitySim = velocity_sim
        if (velocity_sim < minVelocitySim): minVelocitySim = velocity_sim

        error +=  abs(velocity_dataset-velocity_sim)
    velocity_error = error/(meanSim.shape[0]-1)
    return minVelocitySim,maxVelocitySim,velocity_error

#Calcultes the absolute acceleration error between simulation and dataset as well as max and min accelerations and decelerations of the simulation
def accelerationError(meanSim,meanGroundtruth):
    minAccelerationSim = 1000.0 # start at large number
    maxAccelerationSim = 0.0  # start at small number
    minDeccelerationSim = -1000  # start at large number
    maxDeccelerationSim = 0  # start at small number

    error = 0

    for step in range(meanSim.shape[0]-2):
        acceleration_sim = acceleration(meanSim[step,0:3], meanSim[step+1,0:3], meanSim[step+2,0:3])
        acceleration_dataset = acceleration(meanGroundtruth[step,0:3], meanGroundtruth[step+1,0:3], meanGroundtruth[step+2,0:3])

        if (acceleration_sim > 0 and acceleration_sim > maxAccelerationSim): maxAccelerationSim = acceleration_sim
        if (acceleration_sim > 0 and acceleration_sim < minAccelerationSim): minAccelerationSim = acceleration_sim
        if (acceleration_sim < 0 and acceleration_sim < maxDeccelerationSim): maxDeccelerationSim = acceleration_sim
        if (acceleration_sim < 0 and acceleration_sim > minDeccelerationSim): minDeccelerationSim = acceleration_sim

        error +=  abs(acceleration_dataset-acceleration_sim)

    acceleration_error = error/(meanSim.shape[0]-2)
    return minAccelerationSim,maxAccelerationSim,abs(minDeccelerationSim),abs(maxDeccelerationSim),acceleration_error

#Calcultes the absolute torque error between simulation and dataset as well as max and min torques experienced by the arm's joints in simulation
def torqueError(meanSim,meanGroundtruth):
    error = 0
    minTorqueSim = 1000
    maxTorqueSim = 0
    for step in range(1,meanSim.shape[0]):
        torque_sim = 0
        torque_dataset = 0
        for joint in range(7,13):
            torque_sim += abs(meanSim[step,joint])
            torque_dataset += abs(meanGroundtruth[step,joint])
        error += abs(torque_dataset-torque_sim)

        if(torque_sim > maxTorqueSim): maxTorqueSim = torque_sim
        if(torque_sim < minTorqueSim): minTorqueSim = torque_sim

    return minTorqueSim, maxTorqueSim, error/(meanSim.shape[0]-1)

#Calcultes the absolute contact force error between simulation and dataset as well as max and min torques experienced by the Force/Torque sensor in simulation
def contactForceError(meanSim,meanGroundtruth):
    error = 0
    minForceSim = 1000
    maxForceSim = 0
    for step in range(meanSim.shape[0]):
        force_sim = 0
        force_dataset = 0
        for axis in range(16,19):
            force_sim += abs(meanSim[step,axis])
            force_dataset += abs(meanGroundtruth[step,axis])

        error += abs(force_dataset-force_sim)

        if(force_sim > maxForceSim): maxForceSim = force_sim
        if(force_sim < minForceSim): minForceSim = force_sim

    return minForceSim, maxForceSim, error/meanSim.shape[0]

#Calcultes the absolute moment error between simulation and dataset as well as max and min moments experienced by the Force/Torque sensor in simulation
def contactMomentError(meanSim,meanGroundtruth):
    error = 0
    minMomentSim = 1000
    maxMomentSim = 0
    for step in range(meanSim.shape[0]):
        moment_sim = 0
        moment_dataset = 0
        for axis in range(19,22):
            moment_sim += abs(meanSim[step,axis])
            moment_dataset += abs(meanGroundtruth[step,axis])

        error += abs(moment_dataset-moment_sim)

        if(moment_sim > maxMomentSim): maxMomentSim = moment_sim
        if(moment_sim < minMomentSim): minMomentSim = moment_sim

    return minMomentSim, maxMomentSim, error/meanSim.shape[0]

#Calculates the min, max and absolute average acceleration of the object in simulation whilst moving
def objectVelocity(meanSim,meanGroundtruth):
    minVelocitySim = np.ones(20)*1000.0 # start at large number
    maxVelocitySim = np.zeros(20)  # start at small number
    averageVelocity = np.zeros(20)

    start_frame,stop_frame, mean_time = movingTime(meanSim)

    for repeat in range(0,20):
        start_point = int(start_frame[repeat])
        end_point = int(stop_frame[repeat])
        col_start = (repeat*7)+22
        cumulative_velocity = 0
        for step in range(start_point, end_point):
            velocity_sim = velocity(meanSim[step+1,col_start:col_start+3],meanSim[step,col_start:col_start+3])

            if (velocity_sim > maxVelocitySim[repeat]): maxVelocitySim[repeat] = velocity_sim
            if (velocity_sim < minVelocitySim[repeat]): minVelocitySim[repeat] = velocity_sim

            cumulative_velocity += velocity_sim

        if (end_point-start_point) != 0:
            averageVelocity[repeat] = cumulative_velocity/(end_point-start_point)
        else:
            averageVelocity[repeat]=0

    mean_average = np.mean(averageVelocity)
    mean_max = np.mean(maxVelocitySim)
    mean_min = np.mean(minVelocitySim)

    return mean_min, mean_max, mean_average

#Calculates the min, max and absolute average acceleration and deceleration of the object in simulation whilst moving
def objectAcceleration(meanSim,meanGroundtruth):
    minAccelerationSim = np.ones(20)*1000.0 # start at large number
    maxAccelerationSim = np.zeros(20)  # start at small number
    minDecelerationSim = np.ones(20)*-1000.0 # start at large number
    maxDecelerationSim = np.zeros(20)  # start at small number
    averageAcceleration= np.zeros(20)

    start_frame,stop_frame, mean_time = movingTime(meanSim)

    for repeat in range(0,20):
        start_point = int(start_frame[repeat])
        end_point = int(stop_frame[repeat])
        col_start = (repeat*7)+22
        cumulative_acceleration = 0
        for step in range(start_point, end_point-1):

            acceleration_sim = acceleration(meanSim[step,col_start:col_start+3],meanSim[step+1,col_start:col_start+3],meanSim[step+2,col_start:col_start+3])

            if (acceleration_sim > 0 and acceleration_sim > maxAccelerationSim[repeat]): maxAccelerationSim[repeat] = acceleration_sim
            if (acceleration_sim > 0 and acceleration_sim < minAccelerationSim[repeat]): minAccelerationSim[repeat] = acceleration_sim
            if (acceleration_sim < 0 and acceleration_sim < maxDecelerationSim[repeat]): maxDecelerationSim[repeat] = acceleration_sim
            if (acceleration_sim < 0 and acceleration_sim > minDecelerationSim[repeat]): minDecelerationSim[repeat] = acceleration_sim

            cumulative_acceleration += abs(acceleration_sim)
        if (end_point-start_point) != 0:
            averageAcceleration[repeat] = cumulative_acceleration/(end_point-start_point)
        else:
            averageAcceleration[repeat] = 0

    mean_average = np.mean(averageAcceleration)
    mean_max_acceleration = np.mean(maxAccelerationSim)
    mean_min_acceleration = np.mean(minAccelerationSim)
    mean_max_deceleration = np.mean(maxDecelerationSim)
    mean_min_deceleration = np.mean(minDecelerationSim)

    return mean_min_acceleration, mean_max_acceleration, abs(mean_min_deceleration), abs(mean_max_deceleration), mean_average

#Calculates the moving time of the object. Uses the object's velocity and a threshold.
def movingTime(statistics):
    global frequency
    velocity_threshold = 0.01
    start_frame = np.ones(20)*(statistics.shape[0]+5)
    stop_frame = np.zeros(20)
    moving_time = np.zeros(20)

    for repeat in range(0,20):
        col_start = (repeat*7)+22
        for step in range(statistics.shape[0]-1):
            step_velocity = velocity(statistics[step,col_start:col_start+3],statistics[step+1,col_start:col_start+3])
            if step_velocity > velocity_threshold and step>2:
                if start_frame[repeat] > step+1:
                    start_frame[repeat] = step+1
                if stop_frame[repeat] < step+1:
                    stop_frame[repeat] = step+1
        if start_frame[repeat] == statistics.shape[0]+5:
            moving_time[repeat] = 0
        elif start_frame[repeat] == 0:
            moving_time[repeat] = 0
        else:
            moving_time[repeat] = (stop_frame[repeat] - start_frame[repeat])*frequency

    mean_time = np.mean(moving_time)

    return start_frame,stop_frame, mean_time

#Calculates the mahalanobis distance between the final position of the object in simulation and the dataset distribution.
def translationDistribution(meanSim,meanGroundtruth):
    pos_dataset = np.zeros((20,3))
    pos_simulation = np.zeros((20,3))

    for repeat in range(0,20):
        col_start = (repeat*7)+22
        pos_simulation[repeat,:] = meanSim[-1,col_start:col_start+3]
        pos_dataset[repeat, :] = meanGroundtruth[-2,col_start:col_start+3]

    mean_pos_simulation = np.mean(pos_simulation,  axis=0)
    mean_pos_dataset = np.mean(pos_dataset, axis=0)
    cov_pos_dataset = np.cov(np.transpose(pos_dataset))
    inv_cov = distance.mahalanobis(mean_pos_simulation,mean_pos_dataset,linalg.inv(cov_pos_dataset))
    return inv_cov


#Calculates the mahalanobis distance between the final rotation (Euler angles) of the object in simulation and the dataset distribution.
def rotationDistribution(meanSim,meanGroundtruth):
    pos_dataset = np.zeros((20,3))
    pos_simulation = np.zeros((20,3))

    for repeat in range(0,20):

        col_start = (repeat*7)+25
        pos_simulation[repeat,:] = EulerfromQuat(meanSim[-1,col_start],meanSim[-1,col_start+1],meanSim[-1,col_start+2],meanSim[-1,col_start+3])
        pos_dataset[repeat, :] = EulerfromQuat(meanGroundtruth[-2,col_start],meanGroundtruth[-2,col_start+1],meanGroundtruth[-2,col_start+2],meanGroundtruth[-2,col_start+3])
Calculate arm metrics
    mean_pos_simulation = np.mean(pos_simulation,  axis=0)
    mean_pos_dataset = np.mean(pos_dataset, axis=0)
    cov_pos_dataset = np.cov(np.transpose(pos_dataset))
    inv_cov = distance.mahalanobis(mean_pos_simulation,mean_pos_dataset,linalg.inv(cov_pos_dataset))
    return inv_cov


def main(argv):
    simulated_task_location = ''
    groundtruth_location = ''
    #Read in arguments from script and exception handling
    try:
        opts, args = getopt.getopt(argv,"hs:d:",["sfile=","dfile="])
    except getopt.GetoptError:
        print('performance_metrics.py -s <simulator_data> -d <dataset_data>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print("""performance_metrics.py -s <simulator_data> -d <dataset_data>\n\n

            <simulator_data> is the location and filename for the
            10 repeat experiments, i.e /home/user/Documents/taskXX_YY.csv with files
            task01_01.csv through to task01_20.csv\n\n

            <dataset_data> is the location and filename of the mean (not raw) data for
            the relevant task downloaded from https://doi.org/10.4225/08/58b8fc0c5f35d\n""")
            sys.exit(2)
        elif opt in ("-s", "--sfile"):
            simulated_task_location = arg
        elif opt in ("-d", "--dfile"):
            groundtruth_location = arg
    #Read simulation and dataset results
    meanSim = readSim(simulated_task_location)
    meanGroundtruth = readGroundtruth(groundtruth_location,meanSim)

    #Calculate arm metrics
    euclideanResult = euclideanError(meanSim,meanGroundtruth)
    rotationResult = rotationError(meanSim,meanGroundtruth)
    poseResult = poseError(meanSim,meanGroundtruth)
    minVelocitySim, maxVelocitySim, velocity_error = velocityError(meanSim,meanGroundtruth)
    minAccelerationSim,maxAccelerationSim,minDeccelerationSim,maxDeccelerationSim,acceleration_error = accelerationError(meanSim,meanGroundtruth)
    minTorqueSim, maxTorqueSim, TorqueError = torqueError(meanSim,meanGroundtruth)
    minForceSim, maxForceSim, forceError = contactForceError(meanSim,meanGroundtruth)
    minMomentSim, maxMomentSim, momentError = contactMomentError(meanSim,meanGroundtruth)

    if meanSim.shape[1] <= 22:
        #IF an experiment without objects
        simResults = [euclideanResult,
                      rotationResult,
                      poseResult,
                      maxVelocitySim, velocity_error,
                      maxAccelerationSim,maxDeccelerationSim,acceleration_error,
                      minTorqueSim, maxTorqueSim,TorqueError,
                      maxForceSim,forceError,
                      maxMomentSim,momentError]
    else:
        #ELSE an experiment with objects calculate object metrics
        minVelocity, maxVelocity, averageVelocity = objectVelocity(meanSim,meanGroundtruth)
        minAcceleration, maxAcceleration, minDeceleration, maxDeceleration, averageAcceleration = objectAcceleration(meanSim,meanGroundtruth)
        time = movingTime(meanSim)[2]
        transDiff = translationDistribution(meanSim,meanGroundtruth)
        rotDiff = rotationDistribution(meanSim,meanGroundtruth)

        simResults = [euclideanResult,
                      rotationResult,
                      poseResult,
                      maxVelocitySim, velocity_error,
                      maxAccelerationSim,maxDeccelerationSim,acceleration_error,
                      minTorqueSim, maxTorqueSim,TorqueError,
                      maxForceSim,forceError,
                      maxMomentSim,momentError,
                      maxVelocity, averageVelocity,
                      maxAcceleration, maxDeceleration, averageAcceleration,
                      time,
                      transDiff,
                      rotDiff]

    simHeading = ['Euclidean Error',
        'Rotation Error',
        'Pose Error',
        'Maximum Velocity', 'Velocity Error',
        'Maximum Acceleration','Maximum Deceleration', 'Acceleration Error',
        'Minimum Torque', 'Maximum Torque','Torque Error',
        'Maximum Contact Force', 'Contact Force Error',
        'Maximum Contact Moment', 'Contact Moment Error',
        'Maximum Object Velocity', 'Average Object Velocity',
        'Maximum Object Acceleration','Maximum Object Deceleration','Object Average Acceleration',
        'Object Moving Time',
        'Object Translation Error',
        'Object Rotation Error']

    #Write to a CSV file
    with open('results.csv', "a") as csv_file:
        writer = csv.writer(csv_file, delimiter=',')
        writer.writerow(simHeading)
        writer.writerow(simResults)


if __name__ == "__main__":
    main(sys.argv[1:])

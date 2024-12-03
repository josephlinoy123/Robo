#! /usr/bin/env python

class PID(object):
    def __init__(self):
        """This class allows for setting of target theta for each of the target joints
        essentially it allows movement of the robotic arm to set positions at timesteps specified in Rotations"""
        self._kp = 4
        self._ki = 0.0
        self._kd = 0.0
        
        
        self._target_theta = 0.0
        self._sampling_time = 0.05
        
        self._theta0 = 0.0
        self._thetai = 0.0

    def init_status(self):
        self._theta0 = 0.0
        self._thetai = 0.0
    
    def set_target_theta(self, theta):
        self._target_theta = theta
        
    def get_target_theta(self):
        return self._target_theta

    def get_velocity(self, theta):
        """Return the angular velocity of the target joint"""
        error = self._target_theta - theta

        self._thetai += error * self._sampling_time

        dtheta = (error - self._theta0) / self._sampling_time
        self._theta0 = error
        
        
        duty_ratio = (error * self._kp + self._thetai * self._ki + dtheta * self._kd)
        
        if duty_ratio > 30:
            duty_ratio = 30
        elif duty_ratio < -30:
            duty_ratio = -30
        
        return duty_ratio
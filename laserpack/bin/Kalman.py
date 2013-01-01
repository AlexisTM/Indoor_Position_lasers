#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
Kalman.py 

Kalman.py is an implementation of the 1D Kalman filter to get a mean 
and an implementation of a custom 4D Kalman filter to track position 
& heading of the UAV

This file is part of ILPS (Indoor Laser Positioning System).

ILPS is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

ILPS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with ILPS.  If not, see <http://www.gnu.org/licenses/>.

Software created by Alexis Paques and Nabil Nehri for the UCL 
in a Drone-Based Additive Manufacturing of Architectural Structures 
project financed by the MIT Seed Fund

Copyright (c) Alexis Paques 2016
"""


from __future__ import division
from copy import deepcopy

# Static 1D Filter
# I want the mean FAST
class KalmanStatic1D: 
  def __init__(self, _ErrorMeasurement, _ErrorEstimate, _InitialMeasurement, _InitialEstimate = None):
    self.ErrorMeasurement = _ErrorMeasurement
    self.ErrorEstimate = _ErrorEstimate
    self.InitialMeasurement = _InitialMeasurement
    if(_InitialEstimate == None):
      self.PreviousEstimate = _InitialMeasurement
    else :
      self.PreviousEstimate = _InitialEstimate
    # Initiate everythings 
    self.next(_InitialMeasurement)
    
  def next(self, Measurement):
    KalmanGain = self.ErrorEstimate / (self.ErrorEstimate + self.ErrorMeasurement)
    self.PreviousEstimate = self.PreviousEstimate + KalmanGain * (Measurement - self.PreviousEstimate)
    self.ErrorEstimate = (1-KalmanGain)*(self.ErrorEstimate)
    return self.PreviousEstimate

  def get(self):
    return self.PreviousEstimate


# Dynamic optimized 3D Filter with 6 lasers to get X, Y, Z, yaw !
class Custom3DKalman:
    def __init__(self, _ErrorMeasurements, _Error_Yaw, _InitialPrediction, process_covariance_noise_distance=0.01 , process_covariance_noise_angle=0.0174533):
        # Xk is the output value (and the initial value X0 in this case)
        # Should be : (X, Y, Z, Phi(yaw))
        # Should be : (meters, meters, meters, radians)
        self.Xk = _InitialPrediction
        
        # Measurments covariance matrix, imprecision on the lasers
        # Represents the DIAGONAL of the matrix
        self.R = [_ErrorMeasurements*_ErrorMeasurements, \
                   _ErrorMeasurements*_ErrorMeasurements, \
                   _ErrorMeasurements*_ErrorMeasurements, \
                   _Error_Yaw*_Error_Yaw] 

        # This is the Process covariance matrix
        # => 2cm for distances & 5 degrees for angle
        # Represents the DIAGONAL of the matrix
        self.Pk = [_ErrorMeasurements*_ErrorMeasurements, \
                   _ErrorMeasurements*_ErrorMeasurements, \
                   _ErrorMeasurements*_ErrorMeasurements, \
                   _Error_Yaw*_Error_Yaw] 

        # The Process Covariance NOISE is some additionnal error which is unpredictable or not modelised
        # => Walls are unprecised, could be X cm of error 
        # => 1 cm for distances &  1 degree (0,0523599 radians) for angle
        # Represents the DIAGONAL of the matrix
        # Represents the DIAGONAL of the matrix
        self.Q = [process_covariance_noise_distance*process_covariance_noise_distance, \
                   process_covariance_noise_distance*process_covariance_noise_distance, \
                   process_covariance_noise_distance*process_covariance_noise_distance, \
                   process_covariance_noise_angle*process_covariance_noise_angle] 

    def first(self, Measurements):
        self.Xk = [0.0, 0.0, 0.0, 0.0]
        self.Xk[0] = (Measurements[0] + Measurements[1])/2 # X
        self.Xk[1] = (Measurements[2] + Measurements[3])/2 # Y
        self.Xk[2] = (Measurements[4] + Measurements[5])/2 # Z
        self.Xk[3] = (Measurements[6] + Measurements[7])/2 # yaw

    # Angular speed is in radians/s
    # Speed in m/s
    # Acceleration in m/(s*s)
    # Measures in meters
    def next(self, Measurements, Linear_speeds, Linear_accelerations, angular_speed, dt):
        # Predicted value
        dt2 = dt*dt
        # Position
        Xkp = [0.0, 0.0, 0.0, 0.0]
        Xkp[0] = self.Xk[0] + Linear_speeds[0]*dt + Linear_accelerations[0]*dt2
        Xkp[1] = self.Xk[1] + Linear_speeds[1]*dt + Linear_accelerations[1]*dt2
        Xkp[2] = self.Xk[2] + Linear_speeds[2]*dt + Linear_accelerations[2]*dt2
        Xkp[3] = self.Xk[3] + angular_speed*dt

        # Process Covariance Matrix is the precedent Process Covariance + noise
        Pkp = [0.0, 0.0, 0.0, 0.0]
        Pkp[0] = self.Pk[0] + self.Q[0]
        Pkp[1] = self.Pk[1] + self.Q[1]
        Pkp[2] = self.Pk[2] + self.Q[2]
        Pkp[3] = self.Pk[3] + self.Q[3]

        # Kalman Filter
        K = [0.0, 0.0, 0.0, 0.0]
        K[0] = Pkp[0] / (Pkp[0] + self.R[0])
        K[1] = Pkp[1] / (Pkp[1] + self.R[1])
        K[2] = Pkp[2] / (Pkp[2] + self.R[2])
        K[3] = Pkp[3] / (Pkp[3] + self.R[3])

        # Input of measures, improving values cause we got 2 lasers for each directions
        # 6 equations with 4 unknonw
        Yk = [0.0, 0.0, 0.0, 0.0]
        Yk[0] = (Measurements[0] + Measurements[1])/2 # X
        Yk[1] = (Measurements[2] + Measurements[3])/2 # Y
        Yk[2] = (Measurements[4] + Measurements[5])/2 # Z
        Yk[3] = (Measurements[6] + Measurements[7])/2 # yaw

        # New output value taking account the Kalman gain
        self.Xk[0] = Xkp[0] + K[0]*(Yk[0] - Xkp[0])
        self.Xk[1] = Xkp[1] + K[1]*(Yk[1] - Xkp[1])
        self.Xk[2] = Xkp[2] + K[2]*(Yk[2] - Xkp[2])
        self.Xk[3] = Xkp[3] + K[3]*(Yk[3] - Xkp[3])

        # Update the covariance matrix
        self.Pk[0] = (1-K[0])*Pkp[0]
        self.Pk[1] = (1-K[1])*Pkp[1]
        self.Pk[2] = (1-K[2])*Pkp[2]
        self.Pk[3] = (1-K[3])*Pkp[3]
        return self.Xk, K, Xkp

def square(array):
  for a in range(len(array)):
    array[a] = array[a]*array[a]
  return array
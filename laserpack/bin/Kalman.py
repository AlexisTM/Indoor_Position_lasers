#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
Kalman.py 

Kalman.py is an implementation of the 1D Kalman filter

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

class Kalman : 
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

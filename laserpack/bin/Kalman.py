""" 
Kalman.py 

Kalman.py is an implementation of the Kalman filter by Alexis Paques (c) 2016
"""

from __future__ import Division

class Kalman : 
  def __init__(self, _ErrorMeasurement, _ErrorEstimate, _InitialMeasurement, _InitialEstimate = None):
    self.ErrorMeasurement = _ErrorMeasurement
    self.ErrorEstimate = _ErrorEstimate
    self.InitialMeasurement = _InitialMeasurement
    if(_InitialEstimate == None):
      self.PreviousEstimate = _InitialMeasurement
    
  def next(self, Measurement):
    KalmanGain = self.ErrorEstimate / (self.ErrorEstimate + self._ErrorMeasurement)
    #This is the new estimation, which will go to the previous estimation
    self.PreviousEstimate = self.PreviousEstimate + KalmanGain * (Measurement - self.PreviousEstimate)
    self.ErrorEstimate = (1-KalmanGain)*(self.ErrorEstimate)
    return self.PreviousEstimate

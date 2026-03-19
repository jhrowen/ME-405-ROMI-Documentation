"""
@file task_follow_line.py
@brief Task fucnction that computes change to motor setpoints for line following.
@details Proportional controller that tracks line centroid and computes changes to to 
motor velocity setpoints to enable ROMI to follow a line. Implemented as a finite state machine.
@author Jake Rowen
@author Quinn Patterson
@date 2026-03-18
"""
from sensor_class import sensor_array
from encoder_class import Encoder
from task_share import Queue,Share
import micropython
from utime        import ticks_us, ticks_diff

## @brief Wait for line following to be enabled
S1_WAIT = micropython.const(1)
## @brief Run line follower
S2_RUN = micropython.const(2)

## @brief Class definition for line following control task
class task_follow_line:
    ## @brief Constructor for line follower
    ## @param sens Reflectance sensor array object
    ## @param dLeft Queue containing change to left motor setpoint
    ## @param dRight Queue containing change to right motor setpoint
    ## @param timeValuesSensor Buffer for storing time data for plotting
    ## @param dataValuesCentroid Buffer for storing centorid data for plotting
    ## @param follow Boolean flag to enable line following
    def __init__(self, sens: sensor_array, dLeft: Queue, dRight: Queue, timeValuesSensor: Queue, dataValuesCentroid: Queue, follow: Share):
        ## @brief sensor array object
        self._sensor_array = sens
        ## @brief proportional gain value
        self._KP = 100
        ## @brief Share for left motor setpoint change
        self._dLeft = dLeft
        ## @brief Share for right motor setpoint change
        self._dRight = dRight
        ## @brief inital time value
        self._startTime = 0
        ## @brief Queue for time value buffer
        self._timeValuesSensor = timeValuesSensor
        ## @brief Queue for centroid value buffer
        self._dataValuesCentroid = dataValuesCentroid
        ## @brief Share for bool flag
        self._follow = follow
        ## @brief state variable for Finite state machine
        self._state = S1_WAIT
    ## @brief Generator function implemented as a finite state machine or following a line using reflectance sensors.
    ## @details
    ## When follow is set to true, gets the centroid location and computes changes to motor velocity setpoints.
    ## Commented out code stores time and centroid values in queues for output in user task.
    ## @return current state of finite state machine
    def run(self):
        while(True):
            ## @brief Wait for line following to be enabled
            if self._state == S1_WAIT:
                if self._follow.get():
                    self._state = S2_RUN
                    self._startTime = ticks_us()
            ## @brief Run line follower until follow share is set to false
            elif self._state == S2_RUN:
                if not self._follow.get():
                    self._state = S1_WAIT
                else:
                    centroid = self._sensor_array.centroid()
                    if centroid is not None:
                        if abs(centroid)>.04:
                            u = centroid*self._KP
                            if self._dLeft.any():
                                    self._dLeft.get()
                            if self._dRight.any():
                                    self._dRight.get()
                            if centroid>0:
                                self._dLeft.put(u)
                                self._dRight.put(-u)
                            else:
                                self._dLeft.put(u)
                                self._dRight.put(-u)
                    # t   = ticks_us()
                    # # Store the sampled values in the queues
                    # if not self._dataValuesCentroid.full():
                    #     if centroid != None:
                    #         self._dataValuesCentroid.put(centroid)
                    #     else:
                    #         self._dataValuesCentroid.put(0.0)
                    # if not self._timeValuesSensor.full():
                    #     self._timeValuesSensor.put(ticks_diff(t, self._startTime))
            yield self._state
                


                    

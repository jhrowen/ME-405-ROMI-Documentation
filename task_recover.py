"""
@file task_recover.py
@brief Task function that handles recovery
@details
Handles recovery maneuvers when a crash event is detected. Runs motors
in reverse and then turns counterclockwise until the reflectance sensors detect
a line.
"""
from task_share import Share
from micropython import const
from ulab import numpy as np
from sensor_class import sensor_array
import math
## @brief Wait for crash event
S0_WAIT = const(0)
## @brief Reverse state
S1_REVERSE = const(1)
## @brief Turn until line is found
S2_TURN = const(2)

## @brief Class definition for recover task function
class task_recover:
    ## @brief Initializer function recovery task
    ## @param crash Bool flag to indicate crash event\
    ## @param recover_done Bool flag to indicate recovery task has completed
    ## @param vL_r Left motor velocity setpoint
    ## @param vR_r Right motor velocity setpoint
    ## @param shareEstimator Bool flag to start state estimator
    ## @param pcStart Bool flag to start pose controller
    ## @param sL Left motor translational distance 
    ## @param sR Right motor translational distance
    ## @param yaw current heading angle
    ## @param sens Sensor array object
    ## @param recover_mot Bool flag for enabling motor recovery operations
    ## @param follow Bool flag for enabling line following
    def __init__(self,crash,recover_done,vL_r,vR_r,shareEstimator,pcStart,sL,sR,yaw,sens,recover_mot,follow):
        ## @brief Boolean share for enabling recovery mode in motor task
        self._recover_mot : Share = recover_mot
        ## @brief Boolean share for detecting crash event
        self._crash : Share = crash
        ## @brief Boolean share for signaling recovery operation is done
        self._recover_done : Share = recover_done
        ## @brief Share containing left motor setpoint
        self._vL : Share = vL_r
        ## @brief Share containing right motor setpoint
        self._vR : Share = vR_r
        ## @brief state variable
        self._state = S0_WAIT
        ## @brief Boolean flag to start/ stop state estimator
        self._shareEstimator : Share = shareEstimator
        ## @brief Boolean flag to start/ stop pose controller
        self._pcStart : Share = pcStart
        ## @brief Share containing left motor translational distance
        self._sL : Share = sL
        ## @brief Share containing right motor translational distance
        self._sR : Share = sR
        ## @brief distance to reverse mm
        self.reverse = 25
        ## @brief target total left encoder value
        self._targetL = 0
        ## @brief target total right encoder value
        self._targetR = 0
        ## @brief gain value
        self._k = 10
        ## @brief Share containing heading angle
        self._yaw : Share = yaw
        ## @brief Shares containing desired yaw
        self._yaw_des = 0
        ## @brief angular gain
        self._kw = 100
        ## @brief reflectance sensor array object
        self._sens : sensor_array = sens
        ## @brief Boolean share to start line following
        self._follow : Share = follow
        ## @brief countdown for reverse timing
        self._count = 13
    ## @brief Recovery Operations State Machine
    ## @details 
    ## When a crash is detected the finite state machine causes the robot to reverse.
    ## The robot then turns and continues turnung until the line sensor detects 
    ## a line. The state machine then signals that recovery operations are done.
    def run(self):
        while True:
            ## @brief wait for crash detection
            if self._state == S0_WAIT:
                if self._crash.get():
                    self._crash.put(False)
                    self._follow.put(False)
                    self._vL.put(0)
                    self._vR.put(0)
                    self._shareEstimator.put(False)
                    self._pcStart.put(False)
                    self._targetL = self._sL.get() - self.reverse
                    self._targetR = self._sR.get() - self.reverse
                    self._recover_mot.put(True)
                    self._state = S1_REVERSE   
            ## Reverse until count hits 0                
            elif self._state == S1_REVERSE:
                vL = -100
                vR = -100
                self._vL.put(vL)
                self._vR.put(vR)
                self._count-=1               
                if self._count == 0:
                    self._count = 13
                    self._state = S2_TURN
            ## @brief Turn until the sensor array detects a centroid
            elif self._state == S2_TURN:

                if self._sens.centroid() is not None:
                        self._vL.put(0)
                        self._vR.put(0)
                        self._recover_done.put(True)
                        self._recover_mot.put(False)
                        self._state = S0_WAIT
                else:
                    self._vL.put(-100)
                    self._vR.put(100)                           
            yield       

## @brief Wrap angle function
## @details
## Wrap angles around so error does not falsely inflate
## @param angle angle to apply wrapping to.
def wrap_angle(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi
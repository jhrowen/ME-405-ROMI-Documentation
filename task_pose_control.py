"""
@file task_pose_control.py
@brief Pose controller for ROMI
@details
Outputs motor commands to enable ROMI to orient toward a desired
initial angle, travel to a pre programmed x,y point and find a final heading orientation.
@author Jake Rowen
@author Quinn Patterson
@date 2026-03-18
"""
from task_share import Share
from micropython import const
from ulab import numpy as np
import math

## @brief Wait state
S0_WAIT = const(0)
## @brief Orient to initial heading
S1_INIT_POSE = const(1)
## @brief Drive to desired point
S2_DRIVE = const(2)
## @brief Orient to final heading
S3_FINAL_POSE = const(3)

# ROMI Pose controller 

## @brief Pose Controller class definition
class pose_control:
    ## @brief Initialzer for pose controller
    ## @param xPos estimated x position
    ## @param yPos estimated y position
    ## @param estPsi estimated current heading
    ## @param vL Share for left motor velocity
    ## @param vR Share for right motor velocity
    ## @param xDes desired x position
    ## @param yDes desired y position
    ## @param psiDes desired final heading
    ## @param pcStart bool flag for enabling pose controller
    def __init__(self, xPos, yPos, estPsi, vL, vR, xDes, yDes, psiDes, pcStart):
        ## @brief Estimated x position share
        self._x : Share = xPos
        ## @brief Estimated y position share
        self._y : Share = yPos
        ## @brief Estimated heading share
        self._psi : Share = estPsi
        ## @brief Commanded left motor velocity share
        self._vL : Share = vL
        ## @brief Commanded right motor velocity share
        self._vR : Share = vR
        ## @brief Desired x position share
        self._xDes : Share = xDes
        ## @brief Desired y position share
        self._yDes : Share = yDes
        ## @brief Desired final angle share
        self._psiDes : Share = psiDes
        ## @brief Bool flag share for enabling line following
        self._pcStart : Share = pcStart
        ## @brief Variable storing intial heading
        self._psi_init = 0
        ## @brief 1st Angular error proportional gain
        self._kpsi = 650
        ## @brief 2nd angular error proportional gain
        self._kpsi2 = 100
        ## @brief linear error proportional gain
        self._kdist = 2.5
        ## @brief desired x position
        self._x_des = 0
        ## @brief desired y position
        self._y_des = 0
        ## @brief desire heading
        self._psi_des = 0
        ## @brief state variable
        self._state = S0_WAIT
    ## @brief Pose control task finite state machine
    ## @details
    ## Pose controller calculates a desired initial heading based on current heading and the
    ## desired x,y coordinates in a local reference frame. Once oriented the controller commands
    ## the motors to drive to the target x and y location, correcting for error in heading.
    ## Once ROMI arrives at the point, the pose controller commands the motors to orient romi at
    ## the final heading angle, with respect to the initial one. pcCounter goes false when the
    ## maneuver ends.
    def run(self):
        while(True):
            if not self._pcStart.get():
                self._state = S0_WAIT
            ## @brief Wait for controller to activate and calculate initial heading
            if self._state == S0_WAIT:
                self._vL.put(0)
                self._vR.put(0)
                if self._pcStart.get():
                    x_act = self._x.get()
                    y_act = self._y.get()
                    if abs(x_act) < 1 and abs(y_act) < 1:
                        self._x_des = self._xDes.get()
                        self._y_des = self._yDes.get()
                        self._psi_des = self._psiDes.get()
                        dx = self._x_des-x_act
                        dy = self._y_des-y_act
                        psi_act = self._psi.get()
                        self._psi_init = np.arctan2(dy,dx)
                        self._state = S1_INIT_POSE
            ## @brief Orient romi with the final point
            elif self._state == S1_INIT_POSE:
                psi_act = self._psi.get()
                # print(f"{psi_act},{self._psi_init}")
                err = wrap_angle(self._psi_init - psi_act)
                v_cmd = self._kpsi*err
                if v_cmd>0:
                    v_cmd = min(max(v_cmd,85),250)
                else:
                    v_cmd = max(min(v_cmd,-85),-250)
                if abs(err) > .01:                   
                    self._vL.put(-v_cmd)
                    self._vR.put(v_cmd)
                else:
                    self._vL.put(0)
                    self._vR.put(0)
                    self._state = S2_DRIVE
            ## @brief Drive to the final point
            elif self._state == S2_DRIVE:
                x_act = self._x.get()
                y_act = self._y.get()
                # print(f"{x_act},{y_act}")
                psi_act = self._psi.get()
                dx = self._x_des - x_act
                dy = self._y_des - y_act
                psi_des = np.arctan2(dy,dx)
                dist = (dx**2 + dy**2)**0.5
                psi_err = wrap_angle(psi_des - psi_act)
                v_forward = min(max(50,dist*self._kdist),250)
                v_steer = psi_err*self._kpsi2
                if dist > 10:
                    self._vL.put(v_forward-v_steer)
                    self._vR.put(v_forward+v_steer)
                else:
                    self._vL.put(0)
                    self._vR.put(0)
                    self._state = S3_FINAL_POSE
            #% @brief Orient romi to the final heading angle
            elif self._state == S3_FINAL_POSE:
                psi_act = self._psi.get()
                # print(f"{psi_act},{self._psi_des}")
                err = wrap_angle(self._psi_des - psi_act)
                v_cmd = self._kpsi*err
                if v_cmd>0:
                    v_cmd = min(max(v_cmd,85),250)
                else:
                    v_cmd = max(min(v_cmd,-85),-250)
                if abs(err) > .03:                   
                    self._vL.put(-v_cmd)
                    self._vR.put(v_cmd)
                else:
                    self._vL.put(0)
                    self._vR.put(0)
                    self._pcStart.put(False)
                    self._state = S0_WAIT
            yield self._state
## @brief Wrap angle function
## @details
## Wrap angles around so error does not falsely inflate
## @param angle angle to apply wrapping to.
def wrap_angle(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi



                    
                

                


""""
@file task_estimate_state.py
@brief State estimation task function
@details
Task function for estimating the state of ROMI using kinematic model. Estimates arclength,
heading, and angular wheel velocities from encoder data, command voltages and IMU readings.
@author Jake Rowen
@author Quinn Patterson
@date 2026-03-18
"""
from IMU_class import IMU
from encoder_class import Encoder
from ulab import numpy as np
from micropython import const
from task_share import Share, Queue

## @brief State 0 wait state
S0_WAIT = const(0)
## @brief State 1 run
S1_RUN = const(1)
## @brief State 2 skip
S2_SKIP = const(2)

#shareEstimator is a share that sets the state of this task accordingly
## @brief State Estimation Task
class state_est_task:
    ## @brief Constructor for state estimation task function
    ## @param sL Left Encoder translational distance
    ## @param sR Right Encoder translational distance
    ## @param voltageL Left motor command voltage
    ## @param voltageR Right motor command voltage
    ## @param yaw Heading from IMU
    ## @param yaw_rate Yaw rate from IMU
    ## @param shareEstimator Bool flag to enable/disable shareEstimator
    ## @param estS Share for estimated arclength
    ## @param estPsi Share for estimated heading
    ## @param estOmegaL Share for estimated angular velocity of left wheel
    ## @param estOmegaR Share for estimated angular velocity of right wheel
    ## @param xPos Share for estimated x position of ROMI
    ## @param yPos Share for estimated y position of ROMI
    ## @param changex Share to change estimator x value
    ## @param changey Share to change estimator y value
    ## @param leftEncoder Left Encoder Object
    ## @param rightEncoder rightEncoder Object
    def __init__(self, sL, sR, voltageL, voltageR, yaw, yaw_rate, shareEstimator, estS, estPsi, estOmegaL, estOmegaR, xPos, yPos, changex,changey,leftEncoder,rightEncoder):
        ## @brief Share for Left encoder translational distance                                                                                                                 
        self.sL : Share = sL
        ## @brief Share for Right encoder translational distance
        self.sR : Share = sR
        ## @brief Share for Left motor voltage
        self.voltageL : Share = voltageL
        ## @brief Share for Right motor voltage
        self.voltageR : Share = voltageR
        ## @brief Share for yaw value
        self.yaw : Share = yaw
        ## @brief Share for yaw_rate value
        self.yaw_rate :Share = yaw_rate
        ## @brief state variable for finite state
        self._state = S0_WAIT
        ## @brief Bool flag for state estimator
        self._shareEst : Share = shareEstimator
        ## @brief Share for estimated arclength
        self._estS : Share = estS
        ## @brief Share for estimated heading
        self._estPsi : Share = estPsi
        ## @brief Share for esttimated Left wheel velocity
        self._estOmegaL : Share = estOmegaL
        ## @brief Share for esttimated Right wheel velocity
        self._estOmegaR : Share = estOmegaR
        ## @brief Share for esttimated x Position
        self.xPos : Share = xPos
        ## @brief Share for esttimated y Position
        self.yPos : Share = yPos
        ## @brief left encoder object
        self._leftEncoder : Encoder = leftEncoder
        ## @brief right encoder object
        self._rightEncoder : Encoder = rightEncoder
        self._encLInitial = 0
        self._encRInitial = 0
        self._headingInitial = 0
        self._xhat = np.zeros((4,1), dtype = np.float) # initialize state vector
        ## @brief A matrix for state estimation calculation
        self._AD = np.array([[0.7427, 0, 0.2494, 0.2494],[0, 0.0061, 0, 0],[-0.1212, 0, 0.3192, 0.3095],[-0.1212, 0, 0.3095, 0.3192]],dtype = np.float)
        ## @brief B matrix for state estimation calculation
        self._BD = np.array([[0.1962, 0.1962, 0.1287, 0.1287, 0, 0],[0, 0, -0.0071, 0.0071, 0.0001, 0.0039],[0.7137, 0.4156, 0.0606, 0.0606, 0, -1.8098],[0.4156, 0.7137, 0.0606, 0.0606, 0, 1.8098]],dtype = np.float)
        ## @brief C matrix for state estimation calculation
        self._CD = np.array([[1, -70, 0, 0],[1, 70, 0, 0],[0, 1, 0, 0],[0, 0, -0.25, 0.25]], dtype = np.float)
        ## @brief D matrix for state estimation calculation
        self._DD = np.array([[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0]], dtype = np.float)
        self._xpos = 0
        self._ypos = 0
        self._changex : Queue = changex
        self._changey : Queue = changey
        self._initialHeading = 0
    ## @brief Generator function for estimating ROMIs state.
    ## @details
    ## Uses sensor data to estimate ROMIs internal state. Resets everything when the state 
    ## estimator is disabled.
    ## @return current state
    def run(self):
        ## @brief generator function implemented as finite state machine for estimating ROMIs state.
        while(True):
            ## @brief Wait for Estimator to be enabled
            if self._state == S0_WAIT:
                if self._shareEst.get() == True:
                    # reset pose
                    self._xpos = 0
                    self._ypos = 0
                    self._xhat = np.zeros((4,1), dtype = np.float)
                    self._leftEncoder.zero()
                    self._rightEncoder.zero()
                    self.sL.put(0)
                    self.sR.put(0)
                    self.yaw_rate.put(0)
                    self._initialHeading = self.yaw.get()
                    self._estPsi.put(0)
                    self._estS.put(0)
                    self.xPos.put(0)
                    self.yPos.put(0)
                    self._state = S2_SKIP
            ## @brief skip one cycle to allow other tasks to update
            elif self._state == S2_SKIP:
                self._state = S1_RUN
            ## @brief Run state estimator and store variables
            elif self._state == S1_RUN:
                if self._changex.any():
                    self._xpos = self._changex.get()
                if self._changey.any():
                    self._ypos = self._changey.get()
                u_star = np.array([[self.voltageL.get()],[self.voltageR.get()],[self.sL.get()],[self.sR.get()],[self.yaw.get()-self._initialHeading],[self.yaw_rate.get()]], dtype = np.float)
                xhat_new = np.dot(self._AD, self._xhat)+np.dot(self._BD, u_star)
                ds = xhat_new[0,0]-self._xhat[0,0]
                self._xpos += ds*np.cos(xhat_new[1,0])
                self._ypos += ds*np.sin(xhat_new[1,0])
                self.xPos.put(self._xpos)
                self.yPos.put(self._ypos)
                self._xhat = xhat_new
                self._estS.put(float(xhat_new[0,0]))
                self._estPsi.put(float(xhat_new[1,0]))
                self._estOmegaL.put(float(xhat_new[2,0]))
                self._estOmegaR.put(float(xhat_new[3,0]))
                if self._shareEst.get() == False:
                    self._state = S0_WAIT
                # if not self.dataValsArc.full():
                #     self.dataValsArc.put(float(xhat_new[0,0]))
                # if not self.dataValsPsi.full():
                #     self.dataValsPsi.put(float(xhat_new[1,0]))
            yield self._state
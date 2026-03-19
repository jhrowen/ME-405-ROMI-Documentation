"""
@file Final_Path.py
@brief Obstacle Course path planning 
@details
Contains the logic necessary for ROMI to coordinate line following, pose control,
recovery and state estimation.
@author Jake Rowen
@author Quinn Patterson
@date 2023-03-17
"""
import micropython
from task_share import Share
from ulab import numpy as np
from sensor_class import sensor_array
from motor_class import Motor

## @brief Wait for user button to be pressed, enable line following
S0_INIT = micropython.const(1)
## @brief Activate pose controller
S1_SECTION1 = micropython.const(2)
## @brief New pose controller targets
S2_SECTION2 = micropython.const(3)
## @brief Reactivate line following when recovery is done
S3_SECTION3 = micropython.const(4)
## @brief Right turn
S4_SECTION4 = micropython.const(5)
## @brief Activate line following
S5_SECTION5 = micropython.const(6)
## @brief Disable line following and set final pose
S6_SECTION6 = micropython.const(7)
## @brief Disable motors and reset when course completed
S7_SECTION7 = micropython.const(8)

## @brief Obstacle course task
## @details
## This class implements a finite state machine that enables ROMI to naviage the obstacle course. 
class Final_Path:
    ## @brief Constructor for final path task
    ## @param start_run Boolean flag to start the course
    ## @param follow Boolean flag to enable line follower
    ## @param xPos Share containing estimated x position
    ## @param yPos Share containing estimated y position
    ## @param psi Share containing estimated heading angele
    ## @param recover_done Boolean flag indicating recover operations are complete
    ## @param xDes Share containing desired x position
    ## @param yDes Share containing desired y position
    ## @param psiDes Share containing desired heading angle
    ## @param pcStart Boolean flag to start/stop pose controller
    ## @param leftMotorSetpoint Share containing setpoint for left motor
    ## @param rightMotorSetpoint Share containing setpoint for right motor
    ## @param shareEstimator Bool flag to start/stop the state estimator
    ## @param sens Reflectance sensor array object
    ## @param recover_mot Boolean flag to enter/exit recovery mode in motor task
    ## @param vL_r Share for left motor setpoint velocity in recovery mode
    ## @param vR_r Share for right motor setpoint velocity in recovery mode
    ## @param motorL left motor object
    ## @param motorR right motot object
    def __init__(self,start_run,follow,xPos,yPos,psi,recover_done,xDes,yDes,psiDes,pcStart,leftMotorSetpoint,rightMotorSetpoint,shareEstimator,sens,recover_mot,vL_r,vR_r,motorL,motorR):
        ## @brief motor recovery bool flag
        self._recover_mot : Share = recover_mot
        ## @brief variable indicating current state
        self._state = S0_INIT
        ## @brief start run boolean flag
        self._start_run : Share = start_run #share to tell overall task to start
        ## @brief line follower bool flag
        self._follow: Share = follow #share to tell motors to run and use line follower
        ## @brief x position share
        self._xPos : Share = xPos
        ## @brief y position share
        self._yPos : Share = yPos
        ## @brief angular position share
        self._psi = psi
        ## @brief recovery operations done bool
        self._recover_done : Share = recover_done
        ## @brief desired x position share
        self._xDes : Share = xDes
        ## @brief desired y position share
        self._yDes : Share = yDes
        ## @brief desired heading share
        self._psiDes : Share = psiDes
        ## @brief start pose controller flag
        self._pcStart : Share = pcStart
        ## @brief Left motor setpoint share
        self._leftMotorSetpoint : Share = leftMotorSetpoint
        ## @brief Right motor setpoint share
        self._rightMotorSetpoint : Share = rightMotorSetpoint
        ## @brief state estimator bool flag
        self._shareEstimator : Share = shareEstimator
        ## @brief sensor array
        self._sens : sensor_array = sens
        ## @brief left motor recovery velocity share
        self._vL_r : Share = vL_r
        ## @brief right motor recovery velocity share
        self._vR_r : Share = vR_r
        ## @brief left motor object
        self._motL : Motor = motorL
        ## @brief right motor object
        self._motR : Motor = motorR
        ## @brief delay counter variable
        self._counter = 400

    ## @brief Cooperative task generator function for obstacle course
    ## @details
    ## Runs ROMI through the obstacle course using different states, transitioning based
    ## on various sensor readings, boolean flags and recovery status. Yields upon completion
    ## of an if branch
    ## @return Current state of the path planning generator.
    def run(self):
        while(True):
            ## @brief Initial state waiting for button press, once set it enables the line follower.
            if self._state == S0_INIT:
                #Start line follower
                if self._start_run.get():
                    self._motL.enable()
                    self._motR.enable()
                    self._start_run.put(False)
                    self._leftMotorSetpoint.put(300)
                    self._rightMotorSetpoint.put(300)
                    self._state = S1_SECTION1
                    self._follow.put(True) #start line follower here instead of next state
            ## @brief Activates pose controller when centroid is lost
            elif self._state == S1_SECTION1:
                #wait for line folower to lose centroid, then activate pose controller
                if self._sens.centroid() == None:
                    self._shareEstimator.put(True)
                    self._follow.put(False)
                    self._xDes.put(200)
                    self._yDes.put(-100)
                    self._psiDes.put(-(np.pi)/2 - 0.5)
                    self._pcStart.put(True)
                    self._state = S2_SECTION2
            ## @brief Sets new desired pose when previous one is reached            
            elif self._state == S2_SECTION2:
                if not self._pcStart.get():
                    self._shareEstimator.put(False)
                    self._xDes.put(550)
                    self._yDes.put(0)
                    self._psiDes.put((np.pi/2))
                    self._pcStart.put(True)
                    self._state = S3_SECTION3
                    # go forward until wall is hit, then let task_recover take over
            ## @brief wait for recovery operations to finish and reengage line following
            elif self._state == S3_SECTION3:
                self._shareEstimator.put(True)
                
                if self._recover_done.get():
                    self._shareEstimator.put(False)
                    self._recover_done.put(False)
                    self._follow.put(True)
                    self._leftMotorSetpoint.put(300)
                    self._rightMotorSetpoint.put(300)
                    self._state = S4_SECTION4
                    # when recover task is done, reactivate line follower and set setpoints
            
            ## @brief Disable line following and turn right when centroid is lost    
            elif self._state == S4_SECTION4:
                # When centroid is lost, turn off follow and turn right
                if self._sens.centroid() == None:
                    self._follow.put(False)
                    self._recover_mot.put(True)
                    self._vL_r.put(200)
                    self._vR_r.put(-200)
                    self._state = S5_SECTION5

            ## @brief Reenable line following    
            elif self._state == S5_SECTION5:
            # when centroid is found, re enable line following
                if self._sens.centroid()!=None:
                    self._vL_r.put(0)
                    self._vR_r.put(0)
                    self._recover_mot.put(False)
                    self._follow.put(True)
                    self._leftMotorSetpoint.put(200)
                    self._rightMotorSetpoint.put(200)
                    while self._counter>0:
                        self._counter-=1
                        yield self._state
                    self._counter = 400
                    self._state = S6_SECTION6

            ## @brief Set final desired position and heading after line is lost.
            elif self._state == S6_SECTION6:
            # when next centroid is lost again disable line following and set final desired positin and heading
                if self._sens.centroid()==None:
                    self._follow.put(False)
                    self._shareEstimator.put(True)
                    self._xDes.put(-300)
                    self._yDes.put(300)
                    self._psiDes.put(0)
                    self._pcStart.put(True)
                    self._state = S7_SECTION7
            ## @brief Disable motors and reset state machine when final pose is reached.
            elif self._state == S7_SECTION7:
                if not self._pcStart.get():
                    self._shareEstimator.put(False)
                    self._motL.disable()
                    self._motR.disable()
                    self._state = S0_INIT
            yield self._state

                


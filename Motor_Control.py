"""
@file Motor_Control.py
@brief Motor Controller class
@details
Takes a motor, encoder and Proportional/ Integral gain as arguments. Uses a P+I control
loop to adjust motor effort based on a desired linear speed.
@author Jake Rowen
@author Quinn Patterson
@date 2026-03-18
"""
from time import ticks_us, ticks_diff
import encoder_class
import motor_class

## @brief Motor Control Class
class Motor_Control:
    ## @brief Constructor for motor controller object
    ## @param motor Motor object
    ## @param encoder Encoder object
    ## @param KP proportional gain
    ## @param KI integral gain
    def __init__(self, motor, encoder, KP, KI):
        self.state = 0
        self.KP = KP
        self.KI = KI
        self.motor = motor
        self.encoder = encoder
        ## @brief error term from previous time step
        self.e_prev = 0
        self.prev_time = ticks_us()
        self.dt = 0
        self.I = 0
        self.Icurrent = 0
        self.Iflg = True
        ## @brief error term from current time step
        self.err = 0
    ## @brief Control Method
    ## @param Vref reference velocity in mm/s
    ## @return commanded motor effort
    def control(self, Vref):
        # Max velocity is ~ 880 mm/s
        Vref = max(min(Vref, 880), -880)
        self.encoder.update()
        time = ticks_us()
        Vact = self.encoder.get_velocity() # velocity in mm/s
        self.err = Vref-Vact
        # Proportional Effort
        P = self.KP*self.err
        # Integral Effort
        self.dt = ticks_diff(time, self.prev_time)/1000000
        if self.Iflg:
            self.I += self.err*self.dt
            self.Icurrent = self.I*self.KI
        Effort = max(min(P+self.Icurrent, 100), -100)
        if abs(Effort)>=100:
            self.Iflg = False
        else: 
            self.Iflg = True
        self.e_prev = self.err
        self.prev_time = time
        return Effort
    ## @brief Reset error and integral term
    def reset(self):
        self.I = 0
        self.err = 0
        self.Iflg = True
    ## @brief method to change P gain
    ## @param KP_new New KP setpoint
    def change_gain_KP(self, KP_new):
        self.KP = KP_new
    ## @brief Method to change integral gain
    ## @param KI_new New KI gain
    def change_gain_KI(self, KI_new):
        self.KI = KI_new


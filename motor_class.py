"""
@file motor_class.py
@brief File containing Motor class object
@details
Class definition containing initializer and methods to set motor effort, enable and disable
motors and get command voltage.
@author Jake Rowen
@author Quinn Patterson
@date 2026-03-18
"""
from pyb import Pin
from pyb import Timer

## @brief A motor driver interface encapsulated in a Python class. 
## @details 
## Works with motor drivers using separate PWM and direction inputs such as the DRV8838
## drivers present on the Romi chassis from Pololu.
class Motor:
    ## @brief initializer for Motor object
    ## @param PWM Pulse width modulation pin
    ## @param DIR Direction pin
    ## @param nSlp Not sleep pin
    ## @param timer Timer object
    ## @param cnl Channel for motor on timer object
    def __init__(self, PWM, DIR, nSLP, timer, cnl):
        '''Initializes a Motor object'''
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        self.PWM_pin =  timer.channel(cnl, Timer.PWM, pin=Pin(PWM), pulse_width_percent=0)
        self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP, value=0)
        ## @brief Motor effort variable 
        self._effort = 0
        self.nSLP_pin.low()
        self.PWM_pin.pulse_width_percent(0)
    ## @brief Set the motor effort (-100 to 100)
    ## @param effort Commanded motor effort
    def set_effort(self, effort):
        effort = max(min(effort, 100), -100)
        self._effort = effort
        if effort >= 0:
            self.DIR_pin.low()
            self.PWM_pin.pulse_width_percent(effort)
        else:
            self.DIR_pin.high()
            self.PWM_pin.pulse_width_percent(-effort)
    ## @brief Enable Motor
    def enable(self):
        '''Enables the motor driver by taking it out of sleep mode into brake mode'''
        self.nSLP_pin.high()
        self.PWM_pin.pulse_width_percent(0)
    ## @brief Disable Motor
    def disable(self):
        '''Disables the motor driver by taking it into sleep mode'''
        self.nSLP_pin.low()
        self.PWM_pin.pulse_width_percent(0)
    ## @param Get commanded voltage
    def get_voltage(self):
        return self._effort / 100 * 4.5

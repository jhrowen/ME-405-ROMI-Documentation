"""
@file task_read_sensor.py
@brief Task that periodically reads sensor data
@details
Updates sensor data periodically and publishes it to shares for use with many other tasks.
@author Jake Rowen
@author Quinn Patterson
@date 2026-03-18
"""
from IMU_class import IMU
from encoder_class import Encoder
from task_share import Share
from micropython import const
from motor_class import Motor

## @brief Run state
S1_RUN = const(1)

## @brief read sensor task function definition
class read_sensors:
    ## @brief Initializer for read sensors task function
    ## @param IMU IMU object
    ## @param EncoderL Left motor encoder object
    ## @param EncoderR Right motor encoder object
    ## @param MotorL Left motor object
    ## @param MotorR Right motor object
    ## @param sL Left motor enoder translation distance
    ## @param sR Right motor encoder translational distance
    ## @param yaw yaw measured by IMU
    ## @param yaw_rate yaw rate measured by IMU
    ## @param voltageL left motor command voltage
    ## @param voltageR right motor command voltage
    def __init__(self, IMU, EncoderL, EncoderR, MotorL, MotorR, sL, sR, yaw, yaw_rate, voltageL, voltageR):
        ## @brief IMU object
        self.IMU : IMU = IMU
        ## @brief Left Encoder Object
        self.EncoderL : Encoder = EncoderL
        ## @brief Right Encoder Object
        self.EncoderR : Encoder = EncoderR
        ## @brief Left Motor Object
        self.MotorL : Motor = MotorL
        ## @brief Right Motor Object
        self.MotorR: Motor = MotorR
        ## @brief Left Motor translational displacement share
        self.sL : Share = sL
        ## @brief Right Motor translational displacement share
        self.sR : Share = sR
        ## @brief Yaw share
        self.yaw : Share = yaw
        ## @brief Yaw rate share
        self.yaw_rate : Share = yaw_rate
        ## @brief Left motor voltage share
        self.voltageL : Share = voltageL
        ## @brief Right motor voltage share
        self.voltageR : Share = voltageR
        ## @brief state variable
        self._state = S1_RUN
    ## @brief Generator task updates sensor readings
    ## @details
    ## Periodically updates sensor readings and publishes them to shares.
    ## Shares are used by other tasks to avoid pulling data repeatedly in other tasks.
    def run(self):
        while(True):
            # Runs one update of encoder, IMU and voltage data
            if self._state == S1_RUN:
                self.sL.put(self.EncoderL.get_position())
                self.sR.put(self.EncoderR.get_position())
                self.yaw.put(self.IMU.euler()[0])
                self.yaw_rate.put(self.IMU.ang_v()[2])
                self.voltageL.put(self.MotorL.get_voltage())
                self.voltageR.put(self.MotorR.get_voltage()) 
            yield self._state

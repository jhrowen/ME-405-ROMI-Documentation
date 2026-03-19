"""
@file task_sensor.py
@brief Task to calibrate sensor when prompted by the UI
@details
Contains the logic necessary to calibrates the sensor array.
Program can set a minimum value (0) for white through the zero share request,
set a maximum value for black (1) through the normalize share request,
and print values when run share is requested.
@author Jake Rowen
@author Quinn Patterson
@date 2023-03-18
"""

from sensor_class import sensor_array
from task_share   import Share, Queue
from utime        import ticks_us, ticks_diff
from pyb import Pin
import micropython

## @brief state does nothing, was left in case we needed to add anything
S0_INIT = micropython.const(0) # State 0 - initialiation
## @brief state that checks for flags set by the user task and executes them when prompted
S1_READ = micropython.const(1) # State 1 - read values

## @brief sensor task
class task_sensor:
    ## @brief Initializes a sensor task object
    ## @param setzeros (share): boolean to tell line sensor to zero itself
    ## @param setnormalize (share): boolean to tell line sensor to normalize values
    ## @param read (share): boolean to tell sensor to output its data to the terminal
    def __init__(self,sensors,setzeros,setnormalize,read):

        ## @brief initialize state variable
        self._state: int = S0_INIT
        ## @brief initialize sensor array
        self._sensor_array = sensors
        ## @brief initialize setzeros share
        self._setzeros: Share = setzeros #Share to set zeros on line sensor
        ## @brief initialize setnormalize share
        self._setnormalize: Share = setnormalize #Share to set normal high value in line sensor
        ## @brief initialize read share
        self._read: Share = read
        ## @brief initializes centroid variable
        self.centroid = 0
        ## @brief tells user when the initializer has finished
        print("Sensor object instantiated")
    
    ## @brief Cooperative task generator function for sensor task
    def run(self):
        while True:
            ## @brief empty state
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                self._state = S1_READ
            ## @brief checks for if shares are set and relays info to line sensor, prints values when requested
            if self._state == S1_READ: # Init state (can be removed if unneeded)
                if self._setzeros.get():
                    self._sensor_array.zero()
                    self._setzeros.put(False)
                elif self._setnormalize.get():
                    self._sensor_array.normalize()
                    self._setnormalize.put(False)
                elif self._read.get():
                    vals = self._sensor_array.readlevel()
                    c = self._sensor_array.centroid()
                    print(f"{vals[0]},{vals[1]},{vals[2]},{vals[3]},{vals[4]},{c}")
                    self._read.put(False)
            yield self._state
            
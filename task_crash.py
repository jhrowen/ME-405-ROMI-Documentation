"""
@file task_crash.py
@brief Interrupt to detect crash event and handle switch bounce.
@details
Task that handles switch bounce and interrupt that sets crash flag to true when a crash event
is detected.
@author Jake Rowen
@author Quinn Patterson
@date 2026-03-18
"""
from pyb import ExtInt, Pin
from task_share import Share

## @brief Crash Task and interrupt
class task_crash():
    ## @brief Constructor for crash task
    ## @param pin ExtInt pin for crash detection switch
    ## @param Bool flag for crash event
    ## @brief Recovery operations task object
    def __init__(self, pin, crash, recover_task):
        # pin used is PA15
        # Initialize interrupt pin, crash share
        self._switch = ExtInt(pin,ExtInt.IRQ_FALLING,Pin.PULL_UP,self.callback)
        self.crash : Share = crash
        self._recover = recover_task
        ## @brief Variable that counts down until interrupts can be reenabled
        self._countdown = 0
    ## @brief Timer callback for interrupt when crash is detected
    def callback(self,_):
        self._switch.disable()
        self.crash.put(True, in_ISR=True)
        self._countdown = 200
    ## @brief Function generator task that disables interrupt generation for 200 iterations
    ## after an interrupt
    def run(self):
        while True:
            if self._countdown>0:
                self._countdown -= 1
                if self._countdown == 0:
                    self._switch.enable()
            yield
"""
@file encoder_class.py
@brief File containing the Encoder class and its methods
@details 
This class takes a timer and 2 pins as arguments. Its contains methods update,
get_velocity, get_position and zero.
@author Jake Rowen
@author Quinn Patterson
@date 2026-03-17
"""
from pyb import Pin
from pyb import Timer
from time import ticks_us, ticks_diff   # Use to get dt value in update()

## @brief Quadrature Encoder Interface
## @details 
## Uses hardware timer in encoder mode to track position and velocity.
class Encoder:
    ## @brief Encoder object constructor
    ## @details
    ## Intializes an encoder object taking a timer, and channel A and B pins as arguments 
    def __init__(self, timer, chA_pin, chB_pin):
        '''Initializes an Encoder object'''
        self.encoderTimer = Timer(timer, period = 0xFFFF, prescaler = 0)
        self.encoderTimer.channel(1, pin=Pin(chA_pin), mode = Timer.ENC_AB)
        self.encoderTimer.channel(2, pin=Pin(chB_pin), mode = Timer.ENC_AB)
        ## @brief Accumulated position of encoder
        self.position   = 0
        ## @brief Counter value from the most recent update
        self.prev_count = self.encoderTimer.counter()
        ## @brief Change in count between updates
        self.delta      = 0     # 
        ## @brief Change in time between updates
        self.dt         = 0 
        self.prev_time  = ticks_us()

        # track overflow errors
        self.AR = 65535
        self.overflow = -(self.AR+1)/2
        self.underflow = (self.AR+1)/2
    ## @brief Update Method
    ## @details
    ## Reads timer count, computes delta, handles over/underflow, and updates position and time.
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        count = self.encoderTimer.counter()
        time = ticks_us()
        delta = count-self.prev_count
        if delta < self.overflow:
            delta += (self.AR+1)
        elif delta > self.underflow:
            delta -= (self.AR+1)

        self.position += delta
        self.delta = delta
        self.dt = ticks_diff(time, self.prev_time)/1000000
        
        self.prev_time = time
        self.prev_count = count
    ## @brief Read Position
    ## @details
    ## Returns position in linear [mm] units
    ## @return Translational distance in mm
    def get_position(self):
        return self.position*0.153035133 # Convert encoder counts to mm
    ## @brief Compute Velocity
    ## @details
    ## Computes linear velocity based on most recent change in encoder counts vs time and
    ## converts to mm
    ## @return Translational velocity in mm/s  
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        if self.dt == 0:
            return 0
        return (self.delta/self.dt)*0.153035133 # Convert to linear velocity in mm/s
    ## @brief Zero Encoders
    ## @details
    ## Sets encoder positions to zero
    def zero(self):
        self.position = 0
        self.prev_count = self.encoderTimer.counter()
        self.delta = 0
        self.dt = 0
        self.prev_time = ticks_us()
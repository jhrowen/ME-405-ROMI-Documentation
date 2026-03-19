"""
@file task_motor.py
@brief Task that periodically commands a motor
@details 
This task function executes motor commands periodically and depending on 
what operation is being performed. It can run a step response test, set constants, run closed loop 
control at a fixed speed, line following mode and pre-programmed path mode.
@author Jake Rowen
@author Quinn Patterson
@date 2026-03-18
"""
from motor_class import Motor
from encoder_class import Encoder
from Motor_Control import Motor_Control
from task_share   import Share, Queue
from utime        import ticks_us, ticks_diff
import micropython

## @brief Initialization state
S0_INIT = micropython.const(0) # State 0 - initialiation
## @brief Wait for command state
S1_WAIT = micropython.const(1) # State 1 - wait for go command
## @brief Set contants state
S2_SET  = micropython.const(2) # State 2 - Set constants if they changed
## @brief Run fixed speed closed loop control
S3_RUN  = micropython.const(3) # State 3 - run closed loop control
## @brief Line following mode
S4_FOLLOW  = micropython.const(4) # State 4 - run line follower
## @brief Pre programmed path following
S5_PATH = micropython.const(5) # State 5 - pre-programmed path following state
## @brief Recovery operations motor state
S6_RECOVER = micropython.const(6)


## @brief Task function that commands motors based on a current state and target speed values
class task_motor:
    '''
    A class that represents a motor task. The task is responsible for reading
    data from an encoder, performing closed loop control, and actuating a motor.
    Multiple objects of this class can be created to work with multiple motors
    and encoders.
    '''
    ## @brief Intializer for motor task object
    ## @param mot Motor object
    ## @param enc Encoder object
    ## @param control Motor Controller Object
    ## @param goFlag Bool flag for enabling motors
    ## @param dataValues motor velocity buffer queue
    ## @param timeValues time values buffer queue
    ## @param KP Proportional gain share
    ## @param KI Integral gain share
    ## @param Setpoint Motor velocity setpoint value
    ## @param follow Bool flag for enabling line following
    ## @param delta change to motor velocity share
    ## @param vRef reference velocity for path following
    ## @param pcStart Bool flag for enabling pose controller
    ## @param recover_mot Bool flag for enabling motor recovery mode
    ## @param v_r Velocity setpoint for recovery mode
    def __init__(self,
                 mot: Motor, enc: Encoder, control: Motor_Control, 
                 goFlag: Share, dataValues: Queue, timeValues: Queue,
                 KP: Queue, KI: Queue, Setpoint: Queue,
                 follow: Share, delta: Queue, vRef : Share, pcStart: Share, recover_mot: Share,
                 v_r : Share):
        '''
        Initializes a motor task object
        
        Args:
            mot (motor_driver): A motor driver object
            enc (encoder):      An encoder object
            goFlag (Share):     A share object representing a boolean flag to
                                start data collection
            dataValues (Queue): A queue object used to store collected encoder
                                position values
            timeValues (Queue): A queue object used to store the time stamps
                                associated with the collected encoder data
        '''
        ## @brief Share for recovery velocity
        self._v_r : Share = v_r
        self._recover_mot = recover_mot
        ## @brief State variable for finite state machine
        self._state: int        = S0_INIT    # The present state of the task       
        ## @brief motor object
        self._mot: Motor = mot        # A motor object
        ## @brief Encoder object
        self._enc: Encoder      = enc        # An encoder object
        ## @brief Motor controller object
        self._control: Motor_Control = control
        ## @brief Bool Flag enabling motor
        self._goFlag: Share     = goFlag     # A share object representing a
                                             # flag to start data collection
        ## @brief Buffer queue for encoder positions
        self._dataValues: Queue = dataValues # A queue object used to store
                                             # collected encoder position
        ## @brief Buffer queue for time value
        self._timeValues: Queue = timeValues # A queue object used to store the
                                             # time stamps associated with the
                                             # collected encoder data
        ## @brief Share for proportional gain
        self._KP : Queue = KP
        ## @brief Share for Integral Gain
        self._KI : Queue = KI
        ## @brief Motor velocity setpoint queue
        self._Setpoint : Queue = Setpoint   #Queue object

        self._CurrentSetpoint: float = 0.0  #Physical # to pass
        
        self._startTime: int    = 0          # The start time (in microseconds)
                                             # for a batch of collected data
        ## @brief Share for starting line follower
        self._follow: Share = follow #run line follower
        ## @brief Queue for change in motor velocity setpoint
        self._delta: Queue = delta
        self._u = 0.0
        ## @brief Share for reference velocity along a path
        self._vRefPath : Share = vRef
        ## @brief Share for starting path following
        self._pcStart : Share = pcStart

        print("Motor Task object instantiated")
    ## @brief Task function generator implemented as finite state machine for updating the motors
    ## @details
    ## Updates commanded motor velocities and uses motor controller to change the setpoint for the
    ## motor speed. Yields after one update of the motor.
    ## @return Yields current state of the finite state machine
    def run(self):

        
        while True:
            ## @brief Initialzation state
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                print("Initializing motor task") 
                self._state = S1_WAIT
            ## @brief Wait for motor command state   
            elif self._state == S1_WAIT: # Wait for "go command" state
                if self._recover_mot.get():
                    self._control.reset()
                    self._mot.enable()
                    self._state = S6_RECOVER
                
                elif self._goFlag.get():
                    # print("Starting motor loop")
                    # Capture a start time in microseconds so that each sample
                    # can be timestamped with respect to this start time. The
                    # start time will be off by however long it takes to
                    # transition and run the next state, so the time values may
                    # need to be zeroed out again during data processing.
                    self._startTime = ticks_us()
                    self._state = S2_SET
                elif self._follow.get():
                    self._state = S2_SET
                elif self._pcStart.get():
                    self._control.reset()
                    self._mot.enable()
                    self._state = S5_PATH
                
            ## @brief change KP, KI or velocity setpoint
            elif self._state == S2_SET: # Closed-loop control state
                while self._dataValues.any():
                    self._dataValues.get()
                while self._timeValues.any():
                    self._timeValues.get()
                # print("stuck")
                if self._KP.any():
                    # print("changing Kp")
                    self._control.change_gain_KP(self._KP.get())
                if self._KI.any():
                    # print("changing Ki")
                    self._control.change_gain_KI(self._KI.get())
                if self._Setpoint.any():
                    # print("changing setpoint")
                    self._CurrentSetpoint = self._Setpoint.get()
                if self._goFlag.get():
                    self._state = S3_RUN
                elif self._follow.get():
                    self._state = S4_FOLLOW
                self._mot.enable() #make motor able to run
                
            ## @brief Run closed loop motor control at a fixed speeed
            elif self._state == S3_RUN:
                # print(f"Running motor loop, cycle {self._dataValues.num_in()}")
                
                self._mot.set_effort(self._control.control(self._CurrentSetpoint))
                
                # Collect a timestamp to use for this sample
                t   = ticks_us()
                

                
                # Store the sampled values in the queues
                self._dataValues.put(self._enc.get_velocity())
                self._timeValues.put(ticks_diff(t, self._startTime))
                
                # When the queues are full, data collection is over
                if self._dataValues.full():
                    # print("Exiting motor loop")
                    self._control.reset() #reset stored I
                    self._state = S1_WAIT
                    self._goFlag.put(False)
                    self._mot.disable() #make motor unable to run
            ## @brief Line following mode
            elif self._state == S4_FOLLOW:
                if self._delta.any():
                    self._u = self._delta.get()
                
                Vref = self._CurrentSetpoint+self._u

                self._mot.set_effort(self._control.control(Vref))

                if not self._follow.get():
                    self._control.reset() #reset stored I
                    self._state = S1_WAIT
                    self._mot.disable() #make motor unable to run
                    self._follow_started = False
            ## @brief Pre-programmed path following mode
            elif self._state == S5_PATH:
                ref_v = self._vRefPath.get()
                self._mot.set_effort(self._control.control(ref_v))
                if not self._pcStart.get():
                    self._control.reset()
                    self._state = S1_WAIT
                    self._mot.disable()
                    
            ## @brief recovery operations mode
            elif self._state == S6_RECOVER:
                ref_v = self._v_r.get()
                #print(ref_v)
                self._mot.set_effort(self._control.control(ref_v))
                if not self._recover_mot.get():
                    self._control.reset()
                    self._state = S1_WAIT

            yield self._state
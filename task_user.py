"""
@file task_user.py
@brief Task to handle user input given through PUTTY
@details
Contains the logic necessary for ROMI to recieve input from the tethered computer and execute
commands to different tasks. 
Inputs are as follows:
"k" : Allows user to change the Ki and Kp gains for the motor control
"s" : Allows user to change motor setpoint
"g" : Allows user to run motors and outputs both velocity and time data to the serial monitor.
an external script takes in this data and plots a step response
"z" : Zero the line sensor for a given game track
"n" : Trigger the normalize function for a given game track, setting zero then setting the
normalizer lets white be equal to 0 and black be equal to 1 for better sensing
"r" : Outputs reading from each sensor and the centroid of the sensors for debugging
"l" : Runs Romi at given setpoint and activates line follower to apply control and turn Romi, outputs centroid vs time data when finished
"e" : Activates the state estimator
task_user also looks for input from the blue button to initialize the complete path sequence
@author Jake Rowen
@author Quinn Patterson
@date 2023-03-18
"""
import pyb
from pyb import USB_VCP, UART
from task_share import BaseShare,Share, Queue
import micropython
from time import sleep

## @brief Writes initial user prompt and moves onto State 1
S0_INIT = micropython.const(0) # State 0 - initialiation
## @brief Waiting state for user input, sets proper shares or states for a given input
S1_CMD  = micropython.const(1) # State 1 - wait for character input
## @brief Recieves user input to set the motor gains
S2_SETK  = micropython.const(2) # State 2 - wait for gain input
## @brief Recieves user input to set the motor setpoint
S3_SETS  = micropython.const(3) # State 3 - wait for setpoint input
## @brief This state waits after a motor step response is triggered then moves onto state 5
S4_COL  = micropython.const(4) # State 4 - wait for data collection to end
## @brief Takes data from the step response and outputs to the serial monitor 
S5_DIS  = micropython.const(5) # State 5 - display the collected data
## @brief Displays the sensor and centroid data from the line sensor to the serial monitor when prompted
S6_DISC  = micropython.const(6) # State 6 - display the collected data for centroid vs time after line sensor test
## @brief Displays state estimator data to the serial  monitor when prompted
S7_DISP_EST = micropython.const(7) # State 7 - display collected data for state estimator

## @brief Reusable prompt sent to the serial monitor after the ui menu display
UI_prompt = ">: "

## @brief User Task 
class task_user:
   
    ## @brief Initializes a UI task object
    ## @param leftMotorGo (Share):  A share object representing a boolean flag to start data collection on the left motor
    ## @param rightMotorGo (Share): A share object representing a boolean flag to start data collection on the right motor
    ## @param dataValuesleft (Queue):   A queue object used to store collected encoder position values for the left motor
    ## @param timeValuesleft (Queue):   A queue object used to store the time stamps associated with the collected encoder data for the left motor
    ## @param dataValuesright (Queue):   A queue object used to store collected encoder position values for the right motor
    ## @param timeValuesright (Queue):   A queue object used to store the time stamps associated with the collected encoder data for the right motor
    ## @param Left_Motor_KP (Queue): A queue object used to store a new KP value for the left motor
    ## @param Left_Motor_KI (Queue): A queue object used to store a new KI value for the left motor
    ## @param Left_Motor_Setpoint (Queue): A queue object used to store a new Setpoint value for the left motor
    ## @param Right_Motor_KP (Queue): A queue object used to store a new KP value for the right motor
    ## @param Right_Motor_KI (Queue): A queue object used to store a new KI value for the right motor
    ## @param Right_Motor_Setpoint (Queue): A queue object used to store a new Setpoint value for the right motor
    ## @param setzeros (Share): Boolean to tell line sensor to zero itself
    ## @param setnormalize (Share): Boolean to tell line sensor to normalize readings off current reading
    ## @param read (Share): Boolean to tell line sensor to return sensor and centroid data
    ## @param follow (Share): Boolean to tell line sensor to travel at current setpoint and apply its control loop to motors
    ## @param timeValuesSensor (Queue): A queue object that stores time values for line sensor test
    ## @param dataValuesCentroid (Queue): A queue object that stores centroid values for line sensor test
    ## @param shareEstimator (Share): Boolean that turns the state estimator on and off
    ## @param button (Share): Boolean that returns true when the user button has been pressed
    ## @param start_run (Share): Boolean that task_user sends to start Final_Path after button has been pressed
    def __init__(self, LeftMotorGo, RightMotorGo, dataValuesleft, timeValuesleft, dataValuesright, timeValuesright,
                 Left_Motor_KP, Left_Motor_KI, Left_Motor_Setpoint,
                 Right_Motor_KP, Right_Motor_KI, Right_Motor_Setpoint,
                 setzeros, setnormalize, read, follow, timeValuesSensor, dataValuesCentroid,
                shareEstimator, button, start_run):
        

        ## @brief variable indicating current state
        self._state: int          = S0_INIT      # The present state
        ## @brief leftMotorGo share
        self._leftMotorGo: Share  = LeftMotorGo  # The "go" flag to start data
                                                 # collection from both motor pairs
                                                 # motor and encoder pair
        ## @brief rightMotorGo share
        self._rightMotorGo: Share  = RightMotorGo  # The "go" flag to start data
                                                 # collection from both motor pairs
                                                 # motor and encoder pair
        ## @name Serial Output Modes
        # self._ser = UART(3,115_200) ## @brief use this for bluetooth or serial, 3 is btooth and 2 is stlink
        # pyb.repl_uart(self._ser)    ## @brief change repl to go out of btooth
        self._ser: stream         = USB_VCP()    ## @brief A serial port object used to read character entry and to print output
        ## @}
        
        ## @brief dataValuesleft Queue
        self._dataValuesleft: Queue   = dataValuesleft   # A reusable buffer for data
                                                 # collection
        ## @brief timeValuesleft Queue
        self._timeValuesleft: Queue   = timeValuesleft   # A reusable buffer for time
                                                 # stamping collected data
        ## @brief dataValuesright Queue
        self._dataValuesright: Queue   = dataValuesright   # A reusable buffer for data
                                                 # collection
        ## @brief timeValuesright Queue
        self._timeValuesright: Queue   = timeValuesright   # A reusable buffer for time
                                                 # stamping collected data

        ## @name Queues to set left/right motor Kp/Ki/Setpoint values
        self._Left_Motor_KP: Queue   = Left_Motor_KP   ## @brief A reusable buffer for KP left
        self._Left_Motor_KI: Queue   = Left_Motor_KI   ## @brief A reusable buffer for KI left
        self._Left_Motor_Setpoint: Queue   = Left_Motor_Setpoint   ## @brief A reusable buffer for Setpoint left

        self._Right_Motor_KP: Queue   = Right_Motor_KP   ## @brief A reusable buffer for KP right
        self._Right_Motor_KI: Queue   = Right_Motor_KI   ## @brief A reusable buffer for KI right
        self._Right_Motor_Setpoint: Queue   = Right_Motor_Setpoint   ## @brief A reusable buffer for Setpoint right
        ## @}

        ## @name Shares/Queues/Flags to communicate with line follower
        self._setzeros: Share = setzeros ## @brief Share to set zeros on line sensor
        self._setnormalize: Share = setnormalize ## @brief Share to set normal high value in line sensor
        self._read: Share = read ## @brief Share to set read flag for line sensor
        self._follow: Share = follow ## @brief share to tell motors to run and use line follower
        self._timeValuesSensor: Queue = timeValuesSensor 
        self._dataValuesCentroid: Queue = dataValuesCentroid
        self._printcentroidFLG = False
        ## @}

        self._DONE : bool = False # Boolean flag to indicate commands have been receieved
        ## @brief Bool to ensure left motor step response data has printed
        self._printFLGL : bool = False
        ## @brief Bool to ensure right motor step response data has printed
        self._printFLGR : bool = False

        ## @brief shareEstimator Boolean
        self._shareEstimator : Share = shareEstimator
        ## @brief flag to actuate printing of state estimator data
        self._printEstimatorFLG : bool = False

        ## @name Flags to help recieve user input for Kp/Ki/Setpoint
        self.KPbool = False ## @brief boolean to print message to input Kp
        self.KIbool = False ## @brief boolean to print message to input Ki
        self.Setpointbool = False ## @brief boolean to print message to input Setpoint

        self._CharGen = None ## @brief Object for robust user input function
        self._GenFLG = False ## @bref flag to check when user input was finished
        ## @}

        ## @brief button share
        self.button : Share = button
        ## @brief start_run share
        self._start_run : Share = start_run

        ## @brief message to inform user that user task has finished the init function
        self._ser.write("User Task object instantiated\r\n")
    
    ## @brief Cooperative task generator function for user task
    ## @details
    ## Allows Romi to accept user input and send flags to other tasks to perform duties
    ## @return Current state of the user state generator
    def run(self):
        '''
        Runs one iteration of the task
        '''
        while True:
            ## @brief initial state that prints first UI messag and moves to state 1
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                self._ser.write("Initializing user task\r\n")
                self._ser.write("+------------------------------------------------------------------------------+\r\n")
                self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
                self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                self._ser.write("| k | Enter new gain values                                                    |\r\n")
                self._ser.write("| s | Choose a new setpoint                                                    |\r\n")
                self._ser.write("| g | Trigger step response and print results                                  |\r\n")
                self._ser.write("| z | Trigger zero for line sensor                                             |\r\n")
                self._ser.write("| n | Trigger normalize for line sensor                                        |\r\n")
                self._ser.write("| r | Trigger read      for line sensor                                        |\r\n")
                self._ser.write("| l | Run Line Follower                                                        |\r\n")
                self._ser.write("| e | Run State Estimator                                                      |\r\n")
                self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                self._ser.write(UI_prompt)
                self._state = S1_CMD

            ## @brief Initial command state    
            ## @details
            ## State that waits for user input and moves to correct state for the input
            ## "k" goes to state 2
            ## "s" goes to state 3
            ## "g" goes to state 4
            ## "z" , "n" , "r" do not go to states but set flags for line sensor
            ## "l" goes to state 6
            ## "e" goes to state 7
            elif self._state == S1_CMD: # Wait for UI commands               
                ## @brief Wait for at least one character in serial buffer
                if self._ser.any():
                    ## @brief Read the character and decode it into a string
                    inChar = self._ser.read(1).decode()
                    # If the character is an upper or lower case "l", start data
                    # collection on the left motor and if it is an "r", start
                    # data collection on the right motor
                    if inChar in {"g", "G"}:
                        print("GOING")
                        self._ser.write(f"{inChar}\r\n")
                        self._leftMotorGo.put(True)
                        self._rightMotorGo.put(True)
                        self._ser.write("Starting motor loops...\r\n")
                        self._ser.write("Starting data collection...\r\n")
                        self._ser.write("Please wait... \r\n")
                        self._state = S4_COL
                    elif inChar in {"k", "K"}:
                        self._state = S2_SETK
                    elif inChar in {"s", "S"}:
                        self._state = S3_SETS
                    elif inChar in {"z", "Z"}:
                        print("Z set")
                        self._setzeros.put(True)
                    elif inChar in {"n", "N"}:
                        print("N set")
                        self._setnormalize.put(True)
                    elif inChar in {"r", "R"}:
                        self._read.put(True)
                    elif inChar in {"l", "L"}:
                        if self._follow.get():
                            self._follow.put(False)
                            self._state = S6_DISC
                            print("stopping")
                        else:
                            self._follow.put(True)
                            print("following")
                    elif inChar in {"e", "E"}:
                        if self._shareEstimator.get():
                            self._shareEstimator.put(False)
                            self._state = S7_DISP_EST
                        else:
                            self._shareEstimator.put(True)
                elif self.button.get():

                    self.button.put(False)
                    print("User button pressed")
                    if not self._start_run.get():
                        self._start_run.put(True)

            ## @brief sets K values for both motors            
            elif self._state == S2_SETK:
                if not self._Left_Motor_KP.any() and not self._Right_Motor_KP.any():
                    if not self.KPbool:
                        self._ser.write("Set KP Gain")
                        self._ser.write(UI_prompt)
                        self.KPbool = True
                    if self._CharGen is None:
                        self._CharGen = self.getchar(self._Left_Motor_KP,self._Right_Motor_KP)
                        self._GenFLG = True
                    if self._GenFLG:
                        yield from self._CharGen
                elif not self._CharGen == None: 
                    self._CharGen = None

                if not self._Left_Motor_KI.any() and not self._Right_Motor_KI.any():
                    if not self.KIbool:
                        self._ser.write("Set KI Gain")
                        self._ser.write(UI_prompt)
                        self.KIbool = True
                    if self._CharGen is None:
                        self._CharGen = self.getchar(self._Left_Motor_KI,self._Right_Motor_KI)
                        self._GenFLG = True
                    if self._GenFLG:
                        yield from self._CharGen

                ## @brief resends UI message after task is complete
                if self._Left_Motor_KI.any() and self._Right_Motor_KI.any():
                    self._ser.write("+------------------------------------------------------------------------------+\r\n")
                    self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write("| k | Enter new gain values                                                    |\r\n")
                    self._ser.write("| s | Choose a new setpoint                                                    |\r\n")
                    self._ser.write("| g | Trigger step response and print results                                  |\r\n")
                    self._ser.write("| z | Trigger zero for line sensor                                             |\r\n")
                    self._ser.write("| n | Trigger normalize for line sensor                                        |\r\n")
                    self._ser.write("| r | Trigger read      for line sensor                                        |\r\n")
                    self._ser.write("| l | Run Line Follower                                                        |\r\n")
                    self._ser.write("| e | Run State Estimator                                                      |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD
                    self._CharGen = None # set it here instead of in the KI code because it was easier
                    self.KPbool = False
                    self.KIbool = False

            ## @brief sets setpoint for both motors
            elif self._state == S3_SETS:
                if not self._Left_Motor_Setpoint.any() and not self._Right_Motor_Setpoint.any():
                    if not self.Setpointbool:
                        self._ser.write("Set Setpoint in mm/s")
                        self._ser.write(UI_prompt)
                        self.Setpointbool = True
                    if self._CharGen is None:
                        self._CharGen = self.getchar(self._Left_Motor_Setpoint,self._Right_Motor_Setpoint)
                        self._GenFLG = True
                    if self._GenFLG == True:
                        yield from self._CharGen 
                ## @brief reprints UI after setpoint set
                if self._Left_Motor_Setpoint.any() and self._Right_Motor_Setpoint.any():
                    self._ser.write("+------------------------------------------------------------------------------+\r\n")
                    self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write("| k | Enter new gain values                                                    |\r\n")
                    self._ser.write("| s | Choose a new setpoint                                                    |\r\n")
                    self._ser.write("| g | Trigger step response and print results                                  |\r\n")
                    self._ser.write("| z | Trigger zero for line sensor                                             |\r\n")
                    self._ser.write("| n | Trigger normalize for line sensor                                        |\r\n")
                    self._ser.write("| r | Trigger read      for line sensor                                        |\r\n")
                    self._ser.write("| l | Run Line Follower                                                        |\r\n")
                    self._ser.write("| e | Run State Estimator                                                      |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD
                    self._CharGen = None
                    self.Setpointbool = False

            ## @brief state that waits while data is being received
            elif self._state == S4_COL:
                ## @details While the data is collecting (in the motor task) block out the
                ## UI and discard any character entry so that commands don't
                ## queue up in the serial buffer
                if self._ser.any(): self._ser.read(1)
                
                ## @details When both go flags are clear, the data collection must have
                ## ended and it is time to print the collected data.
                if not self._leftMotorGo.get() and not self._rightMotorGo.get():
                    self._ser.write("Data collection complete...\r\n")
                    self._ser.write("Printing data...\r\n")
                    self._state = S5_DIS

            ## @brief state that prints motor step data
            elif self._state == S5_DIS:
                ## @details While data remains in the buffer, print that data in a command
                ## separated format. Otherwise, the data collection is finished.
                if self._printFLGL == False:
                    self._ser.write(f"DIS: left={self._dataValuesleft.num_in()} right={self._dataValuesright.num_in()}\r\n")
                    self._ser.write("--------------------\r\n")
                    self._ser.write("Left Time, Left Position\r\n")
                    self._printFLGL = True
                elif self._dataValuesleft.any():
                    self._ser.write(f"{self._timeValuesleft.get()},{self._dataValuesleft.get()}\r\n")
                elif self._printFLGR == False:
                    self._ser.write("--------------------\r\n")
                    self._ser.write("Right Time, Right Position\r\n")
                    self._printFLGR = True
                elif self._dataValuesright.any():
                    self._ser.write(f"{self._timeValuesright.get()},{self._dataValuesright.get()}\r\n")
                ## @brief reprint UI message
                else:
                    self._ser.write("--------------------\r\n")
                    self._ser.write("\r\n")
                    self._ser.write("+------------------------------------------------------------------------------+\r\n")
                    self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write("| k | Enter new gain values                                                    |\r\n")
                    self._ser.write("| s | Choose a new setpoint                                                    |\r\n")
                    self._ser.write("| g | Trigger step response and print results                                  |\r\n")
                    self._ser.write("| z | Trigger zero for line sensor                                             |\r\n")
                    self._ser.write("| n | Trigger normalize for line sensor                                        |\r\n")
                    self._ser.write("| r | Trigger read      for line sensor                                        |\r\n")
                    self._ser.write("| l | Run Line Follower                                                        |\r\n")
                    self._ser.write("| e | Run State Estimator                                                      |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write(UI_prompt)
                    self._printFLGL = False
                    self._printFLGR = False
                    self._state = S1_CMD

            ## @brief displays sensor data as function of time to track line sensor control response
            elif self._state == S6_DISC:
                if self._printcentroidFLG == False:
                    # self._ser.write("Time, Centroid\r\n")
                    self._printcentroidFLG = True
                if self._dataValuesCentroid.any():
                    self._ser.write(f"{self._timeValuesSensor.get()},{self._dataValuesCentroid.get()}\r\n")
                ## @brief reprint UI message
                else:
                    self._ser.write("--------------------\r\n")
                    self._ser.write("\r\n")
                    self._ser.write("+------------------------------------------------------------------------------+\r\n")
                    self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write("| k | Enter new gain values                                                    |\r\n")
                    self._ser.write("| s | Choose a new setpoint                                                    |\r\n")
                    self._ser.write("| g | Trigger step response and print results                                  |\r\n")
                    self._ser.write("| z | Trigger zero for line sensor                                             |\r\n")
                    self._ser.write("| n | Trigger normalize for line sensor                                        |\r\n")
                    self._ser.write("| r | Trigger read      for line sensor                                        |\r\n")
                    self._ser.write("| l | Run Line Follower                                                        |\r\n")
                    self._ser.write("| e | Run State Estimator                                                      |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD
                    self._printcentroidFLG = False

            ## @brief prints state estimator data, functionality removed to save ram space when debugging was no longer required
            elif self._state == S7_DISP_EST:
                if not self._printEstimatorFLG:
                    # print("Arc Length [mm], Heading Angle [rad]")
                    self._printEstimatorFLG = True
                ## @brief reprint UI message
                else:
                    self._ser.write("--------------------\r\n")
                    self._ser.write("\r\n")
                    self._ser.write("+------------------------------------------------------------------------------+\r\n")
                    self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write("| k | Enter new gain values                                                    |\r\n")
                    self._ser.write("| s | Choose a new setpoint                                                    |\r\n")
                    self._ser.write("| g | Trigger step response and print results                                  |\r\n")
                    self._ser.write("| z | Trigger zero for line sensor                                             |\r\n")
                    self._ser.write("| n | Trigger normalize for line sensor                                        |\r\n")
                    self._ser.write("| r | Trigger read      for line sensor                                        |\r\n")
                    self._ser.write("| l | Run Line Follower                                                        |\r\n")
                    self._ser.write("| e | Run State Estimator                                                      |\r\n")
                    self._ser.write("+---+--------------------------------------------------------------------------+\r\n")
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD
                    self._printEstimatorFLG = False

            yield self._state
    ## @brief Get character inputs function
    ## @details Function to handle multiple character inputs throughout task_user.
    ## Utilized for inputs to set Kp, Ki, and Setpoint.
    ## See multichar_input.py in Lab 0x04 files for commented version of this code, 
    ## some changes were made to this one but the overall structure is the same.
    def getchar(self, outshare, outshare2):
        out_share = outshare
        out_share2 = outshare2
        char_buf: str      = ""
        digits:   set(str) = set(map(str,range(10)))
        term:     set(str) = {"\r", "\n"}
        done = False
        while not done:
            if self._ser.any():
                char_in = self._ser.read(1).decode()
                if char_in in digits:
                    self._ser.write(char_in)
                    char_buf += char_in
                elif char_in == "." and "." not in char_buf:
                    self._ser.write(char_in)
                    char_buf += char_in
                elif char_in == "-" and len(char_buf) == 0:
                    self._ser.write(char_in)
                    char_buf += char_in
                elif char_in == "\x7f" and len(char_buf) > 0:
                    self._ser.write(char_in)
                    char_buf = char_buf[:-1]
                elif char_in in term:
                    self._ser.write("\r\n")
                    if len(char_buf) == 0:
                        self._ser.write("Value not changed\r\n")
                        char_buf = ""
                    elif char_buf not in {"-", "."}:
                        value = float(char_buf)
                        out_share.put(value)
                        out_share2.put(value)
                        self._ser.write(f"Value set to {value}\r\n")
                    done = True
                    self._GenFLG = False
            yield
'''
@file main.py
@brief Main file for ROMI obstacle course navigating robot
@details
This file initializes hardware, shared variables, objects and tasks to be run in the 
cooperative scheduler including:
    - Motor Control
    - State Estimation
    - Line Following
    - User Interface
    - Sensor Updates
    - Recovery Operations
    - Obstacle Course Path Planning
@author Jake Rowen
@author Quinn Patterson
@date 2026-03-17
'''

from pyb import I2C
from time import sleep_ms
from IMU_class import IMU, MODE_IMU
from Motor_Control import Motor_Control
from encoder_class import Encoder
from motor_class import Motor
from task_motor   import task_motor
from pyb import Timer,Pin,ExtInt
from time import sleep_ms, ticks_ms, ticks_diff
from task_user    import task_user
from task_share   import Share, Queue, show_all
from cotask       import Task, task_list
import gc
from gc           import collect
from task_follow_line import task_follow_line
from task_read_sensor import read_sensors
from task_estimate_state import state_est_task
from sensor_class import sensor_array
from task_sensor import task_sensor
from task_crash import task_crash
from task_recover import task_recover
from Final_Path import Final_Path
from task_pose_control import pose_control
#from task_generate_curve import generate_curve
#from task_follow_path import follow_path_task

# --- I2C Setup (I2C2 = PB10/PB11 on your board) ---

## @section hard_init Hardware Initialization

## @brief I2C bus for BNO055 IMU
## @details
## Sets up I2C communication with a 400000 Hz baudrate, creates an IMU Object and
## initializes it in IMU Mode.
i2c = I2C(2)
i2c.init(I2C.CONTROLLER, baudrate=400000)
imu = IMU(i2c, addr=0x28)
imu.initialize(mode=MODE_IMU, reset=True, units=0x06)

## @brief Initialize Timer and motor objects
## @details 
## Initializes timer 3 to use for the motor objects. Creates left and 
## right motor objects.
## - Pin.cpu.A6 Left Motor PWM Pin
## - Pin.cpu.B9 Left Motor Dir Pin
## - Pin.cpu.B8 Left Motor nSleep Pin
## - Pin.cpu.A7 Right Motor PWM Pin
## - Pin.cpu.B5 Right Motor Dir Pin
## - Pin.cpu.B4 Right Motor nSleep Pin

motorTimer = Timer(3, freq=20000)

Left_Motor  = Motor(Pin.cpu.A6, Pin.cpu.B9, Pin.cpu.B8, motorTimer, 1)
Right_Motor = Motor(Pin.cpu.A7, Pin.cpu.B5, Pin.cpu.B4, motorTimer, 2)

## @brief Left and Right Encoder Objects
## @details 
## Creates left and right Encoder objects. Left Encoder operates on 
## timer 1 and right encoder operates on timer 2.
## - Pin.cpu.A8 Left encoder channel A
## - Pin.cpu.A9 Left encoder channel B
## - Pin.cpu.A0 Right encoder channel A
## - Pin.cpu.A1 Right encoder channel B

Left_Encoder  = Encoder(1, Pin.cpu.A8, Pin.cpu.A9)
Right_Encoder = Encoder(2, Pin.cpu.A0, Pin.cpu.A1)

Left_Motor.enable()
Right_Motor.enable()

## @brief Left and right motor controller objects
## @details
## Creates left and right motor controller objects with a KP gain of 0.15 
## KI gain of 0.001

Control_Left = Motor_Control(Left_Motor, Left_Encoder, 0.15, 0.001)
Control_Right = Motor_Control(Right_Motor, Right_Encoder, 0.15, 0.001)

## @brief Reflectance sensor object
## @details
## Instantiates an object for the IR Reflectance sensor array used for line detection.
## - Pin.cpu.B0 Reflectance sensor 1
## - Pin.cpu.B1 Reflectance sensor 2
## - Pin.cpu.C0 Reflectance sensor 3
## - Pin.cpu.A4 Reflectance sensor 4
## - Pin.cpu.C1 Reflectance sensor 5
## - Pin.cpu.C10 Reflectance sensor array control pin

sensors = sensor_array(Pin.cpu.B0,Pin.cpu.B1,Pin.cpu.C0,Pin.cpu.A4,Pin.cpu.C1,Pin.cpu.C10)

## @brief Pin object for crash sensor
## @details
## - Pin.cpu.A15 Crash sensor Pin

pinCrash = Pin.cpu.A15

# Build shares and queues

## @section shared_vars Shared Variables

## @brief Boolean go flags for Left Motor task to start step response test
LeftMotorGo   = Share("B",     name="Go Flag Left")
## @brief Boolean go flags for Right Motor task to start step response test
RightMotorGo  = Share("B",     name="Go Flag Right")
## @brief Queue object for left motor task containing KP
Left_Motor_KP = Queue("f", 1, name="Left Motor KP Value")
## @brief Queue object for left motor task containing KI
Left_Motor_KI = Queue("f", 1, name="Left Motor KI Value")
## @brief Queue object for left motor task containing setpoint
Left_Motor_Setpoint = Queue("f", 1, name="Setpoint for left motor")
## @brief Queue object for right motor task containing KP
Right_Motor_KP = Queue("f", 1, name="Right Motor KP Value")
## @brief Queue object for right motor task containing KI
Right_Motor_KI = Queue("f", 1, name="Right Motor KI Value")
## @brief Queue object for right motor task containing setpoint
Right_Motor_Setpoint = Queue("f", 1, name="Setpoint for right motor")
## @brief Data buffer for left motor task to store motor velocity data
dataValuesleft    = Queue("f", 30, name="Data Collection Buffer left")
## @brief Data buffer for left motor task to store motor time data
timeValuesleft    = Queue("L", 30, name="Time Buffer left")
## @brief Data buffer for right motor task to store motor velocity data
dataValuesright    = Queue("f", 30, name="Data Collection Buffer right")
## @brief Data buffer for right motor task to store motor time data
timeValuesright    = Queue("L", 30, name="Time Buffer right")
## @brief Boolean flag for calibrating white color for reflectance sensor
setzeros = Share("B" , name = "zero setter for line sensor")
## @brief Boolean flag for calibrating black color for reflectance sensor
setnormalize = Share("B" , name="normalizer for line sensor")
## @brief Boolean flag for printing reflectance sensor readings
read = Share("B" , name="read flag for line sensor")
## @brief Boolean flag for enabling line following
follow = Share("B" , name="run line follower")
## @brief Queue containing change to left motor velocity setpoint for line following
dLeft = Queue('f', 1, name="Left Motor setpoint delta")
## @brief Queue containing change to right motor velocity setpoint for line following
dRight = Queue('f', 1, name="Right Motor setpoint delta")
## @brief Data Buffer for collecting time data
timeValuesSensor    = Queue("L", 300, name="Time Buffer left")
## @brief Data Buffer for collecting centroid data
dataValuesCentroid    = Queue("f", 300, name="Data Collection Buffer right")
## @brief Share containing distance covered by left wheel
sL = Share("f", name="Left encoder translational distance [mm]")
## @brief Share containing distance covered by right wheel
sR = Share("f", name="Right encoder translational distance [mm]")
## @brief Share containing voltage commanded to left motor
voltageL = Share("f", name="Left Motor Command Voltage")
## @brief Share containing voltage commanded to right motor
voltageR = Share("f", name="Right Motor Command Voltage")
## @brief Share containing current heading angle
yaw = Share("f", name="Heading angle [rad]")
## @brief Share containing current heading rate
yaw_rate = Share("f", name="Yaw Rate [rad/s]")
## @brief Bool flag for enabling state estimator
shareEstimator = Share("B", name="State Estimation flag")
shareEstimator.put(False)
## @brief Share containing estimated arclength
estS = Share("f",name="Estimate Linear Position")
## @brief Share containing estimated heading
estPsi = Share("f",name="Estimated heading Angle") 
## @brief Share containing estimated angular velocity of left wheel
estOmegaL = Share("f", name="Estimate angular velocity of Left Wheel")
## @brief Share containing estimated angular velocity of right wheel
estOmegaR = Share("f", name="Estimate angular velocity for Right Wheel")
## @brief Boolean flag set when crash is detected
crash = Share('B',name="Boolean Flag to detect crash")
## @brief Boolean flag indicating button press
button = Share('B',name="Bool flag for user button press")
## @brief Share containing estimated x position
xPos = Share('f', name="Estimated X position")
## @brief Share containing estimated y position
yPos = Share('f', name="Estimated Y position")
## @brief Share indicating recovery operations have been completed
recover_done = Share("B", name="Finished Recovery Operations")
# xdot = Share('f', name="translational x velocity")
# ydot = Share('f', name="Translational y velocity")
# start_path = Share('B', name="booloean flag to start path follower")
# ax = Share('f', name="ax coefficient")
# bx = Share('f', name="bx coefficient")
# cx = Share('f', name="cx coefficient")
# dx = Share('f', name="dx coefficient")
# ay = Share('f', name="ay coefficient")
# by = Share('f', name="by coefficient")
# cy = Share('f', name="cy coefficient")
# dy = Share('f', name="dy coefficient")
#segment_id = Share('B', name="Segment ID")
## @brief Share containing linear velocity setpoint for left motor
vL = Share('f', name="Left Motor Velocity")
## @brief Share containing linear velocity setpoint for right motor
vR = Share('f', name="Right Motor Velocity")
# follow_path = Share('B',name='Bool flag for enabling path following')
# next_segment = Share('B',name="Bool flag for starting next segment")
## @brief Boolean flag indicating start of obstacle course sequence
start_run =Share('B',name = "Bool flag for starting the pathing sequence")
## @brief Share containing value to change x estimate from state estimator
changex = Queue('f', 1, name="change state estimator x position")
## @brief Share containing value to change x estimate from state estimator
changey = Queue('f', 1, name="change state estimator y position")
## @brief Share containing target x position
xDes = Share('f', name="desired x position")
## @brief Share containing target y position
yDes = Share('f', name="Desired y position")
## @brief Share containing target final heading
psiDes = Share('f', name="Desired final heading")
## @brief Boolean flag to enable/disable pose controller
pcStart = Share('B', name="Bool flag to start pose controller")
## @brief Boolean flag to enable motor task recovery state
recover_mot = Share('B', name="Bool Flag for enabling recovery ops in motor task")
recover_mot.put(False)
## @brief Share containing setpoint velocity left motor in recovery mode
vL_r = Share('f', name="recovery task left motor velocity")
## @brief Share containing setpoint velocity for right motor in recovery mode
vR_r = Share('f', name="recovery task right motor velocity")

# Build task class objects
## @section create_tasks Create Task Objects
## @brief Left and right motor tasks
## @details
## Tasks intake setpoint velocities and use the motor controller object to update motor commands
## every 35 ms with a priority of 3 in the scheduler. Can also enable step response and data collection.
leftMotorTask  = task_motor(Left_Motor,  Left_Encoder, Control_Left,
                            LeftMotorGo, dataValuesleft, timeValuesleft, 
                            Left_Motor_KP, Left_Motor_KI, Left_Motor_Setpoint,
                            follow, dLeft, vL, pcStart,recover_mot,vL_r)
rightMotorTask = task_motor(Right_Motor, Right_Encoder, Control_Right,
                            RightMotorGo, dataValuesright, timeValuesright,
                            Right_Motor_KP, Right_Motor_KI, Right_Motor_Setpoint,
                            follow, dRight, vR, pcStart,recover_mot, vR_r)
## @brief User interface task
## @details
## Task runs every millisecond with a priority of zero. Enables a user to interact with ROMI through a variety of 
## keyboard inputs. Also responsible for printing out collected data from certain tasks.
userTask = task_user(LeftMotorGo, RightMotorGo, dataValuesleft, timeValuesleft, dataValuesright, timeValuesright,
                     Left_Motor_KP, Left_Motor_KI, Left_Motor_Setpoint,
                     Right_Motor_KP, Right_Motor_KI, Right_Motor_Setpoint,
                     setzeros, setnormalize, read, follow, timeValuesSensor, dataValuesCentroid, shareEstimator, button, start_run)
## @brief Line sensor task
## @details
## Enables a user to interact with the line sensor and calibrate it.
sensor_task = task_sensor(sensors, setzeros, setnormalize,read)
## @brief Line following task
## @details 
## When enabled, calculates changes to the setpoints for left and right motors using centroid data 
## to enable ROMI to follow a line. Has a period of 10 milliseconds and a priority of 2.
line_follow_task = task_follow_line(sensors, dLeft, dRight, timeValuesSensor, dataValuesCentroid, follow)
## @brief Read sensors task
## @details 
## Task that periodically reads sensor data from Encoders, Motors and IMU and publishes it to shares. Operates
## at a period of 3 milliseconds with a priority of 0.
read_sensors_task = read_sensors(imu,Left_Encoder,Right_Encoder,Left_Motor,Right_Motor,sL,sR,yaw,yaw_rate,voltageL,voltageR)
## @brief State Estimation Task
## @details 
## Fuses data from various sensors to provide an estimated state for Romi. Operates every 10 milliseconds with a priority of 1.
estimate_state_task = state_est_task(sL,sR,voltageL,voltageR,yaw,yaw_rate,shareEstimator,estS,estPsi,estOmegaL,estOmegaR,xPos,yPos,changex,changey,Left_Encoder,Right_Encoder)
## @brief Recovery Task
## @details
## High Priority task that handles recovery maneuvers when a crash event is detected. Operates every 20 milliseconds with a
## priority of 5.
recover_obj = task_recover(crash,recover_done,vL_r,vR_r,shareEstimator,pcStart,sL,sR,yaw,sensors,recover_mot,follow)
recover_task = Task(recover_obj.run, name="Crash recovery maneuvers task", priority=5, period=20, profile=True)
## @brief Crash Task
## @details 
## Low priority task for detecting crash events and preventing switch bounce. Runs every 5 milliseconds with 0 priority.
crash_task = task_crash(pinCrash,crash,recover_task)
## @brief Obstacle Course Path Planning Task
## @details
## Task that commands ROMIS movement sequence and order of operations to navigate the obstacle course.
## Operates with a period of 20 milliseconds and a priority of 2.
final_path_task = Final_Path(start_run,follow,xPos,yPos,estPsi,recover_done,xDes,yDes,psiDes,pcStart,Left_Motor_Setpoint,Right_Motor_Setpoint,shareEstimator,sensors,recover_mot,vL_r,vR_r,Left_Motor,Right_Motor)
## @brief Pose Control Task
## @details 
## Task that commands motor speed setpoints based on a desired x and y position and final heading using the state estimator.
## Operates at a period of 10 milliseconds and a priority of 2.
pose_control_task = pose_control(xPos, yPos, estPsi, vL, vR, xDes, yDes, psiDes, pcStart)
#generate_curve_task = generate_curve(estS,estPsi,estOmegaL,estOmegaR,start_path,xPos,yPos,xdot, ydot,ax,bx,cx,dx,ay,by,cy,dy,segment_id,follow_path,next_segment,changex,changey)
#task_follow_path = follow_path_task(vL,vR,ax,bx,cx,dx,ay,by,cy,dy,follow_path,segment_id,next_segment,xPos,yPos,estPsi)

## @brief Task list
## @details
## Initialize all tasks as task objects and append them to the task list for the scheduler
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority = 3, period = 35, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 3, period = 35, profile=True))
task_list.append(Task(sensor_task.run, name="Sensor Task",
                      priority = 2, period = 10, profile = True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 1, profile= True))
task_list.append(Task(line_follow_task.run, name="Line Following Task",
                        priority = 2, period = 10, profile=True))
task_list.append(Task(read_sensors_task.run, name="Read Sensors Task", priority=0, period=3,profile=True))
task_list.append(Task(estimate_state_task.run, name="State Estimation Task", priority=1, period=10,profile=True))
task_list.append(recover_task)
task_list.append(Task(crash_task.run, name="Crash Detection Task", priority=0, period=5, profile=True))
task_list.append(Task(final_path_task.run, name="final movements",priority=2,period=20,profile=True))
task_list.append(Task(pose_control_task.run, name="pose controller",priority=2, period = 10, profile = True))
#task_list.append(Task(generate_curve_task.run, name="Spline Curve Generator",priority=4,period=50,profile=True))
#task_list.append(Task(task_follow_path.run, name="Follow predetermined path task",priority=2,period=10,profile=True))

# Run the garbage collector preemptively
collect()

## @section button_int Button Press Interrupt
## @brief User Button Interrupt Generation
## @details
## Extint callback function for user button press. Generates an interrupt and sets button flag
## true when user button on Nucleo is pressed.
## @param _ unused interrupt callback argument
def callback(_):
    button.put(True)

pinButton = ExtInt(Pin.cpu.C13,ExtInt.IRQ_FALLING,Pin.PULL_UP,callback)
print(gc.mem_free())
# Run the scheduler until the user quits the program with Ctrl-C
## @section scheduler Scheduler
## @brief Scheduler Loop
## @details 
## Cooperative scheduler runs all tasks at the given periods and priorities until a keyboard interrupt is generated.
while True:
    try:
        task_list.pri_sched()
        
    except KeyboardInterrupt:
        print("Program Terminating")
        print("\n=== TASK PROFILE ===")
        print(task_list)          # <--- prints the profiling table
        Left_Motor.disable()
        Right_Motor.disable()
        break



# ME-405-ROMI-Documentation
This repository stores the code for our romi project that was developed over the 10 weeks of the Winter 2026 quarter at California Polytechnic State University, San Luis Obispo. 

Our entire codebase and useful additional files are located here:                                                            
https://github.com/Quig-Beep/ME405-Project-Documentation.git

Documentation for all code we were given for this project can be located below in Dr. Ridgleys own Github repository:
https://github.com/spluttflob/ME405-Support?tab=readme-ov-file

There are a few tabs to navigate on this website. Packages and Classes provide information on our files and the structures within. Clicking on a package or class will bring you to a more detailed view of the task. The file list provides a concise list of all of our files for easy viewing.

A video of Romi running the complete course can be viewed below:

[![Video Title](https://img.youtube.com/vi/zxmrBjntwv4/0.jpg)](https://www.youtube.com/watch?v=zxmrBjntwv4)

A closer look at Romi can also be viewed here:

[![Video Title](https://img.youtube.com/vi/KbessiJycDM/0.jpg)](https://www.youtube.com/watch?v=KbessiJycDM)

Our robot was designed with 4 sensor types to help detect its environment:                                                    
Quadrature encoders - Read encoder counts with hall effect sensors to detect motor movement                                   
Internal Measurement Unit - Gave accelerometer and gyroscope data to calculate accurate headings and feed data to a state estimator to infer X and Y position of Romi                                                                                   
Line Sensor - Read IR reflectance of the ground below romi to determine if it was on top of a line or not                     
Bump Sensor - Simple switch that would trigger an interrupt when closed, alerting Romi that it had hit something

The ROMI Pin Configuration excel spreadsheet located in our codebase repository provides a pinout of our wiring with romi and associated boards. Ideally with the same hardware and this sheet anyone would be able to reproduce our results. The excel sheet also contains links to useful datasheets for the BNO055 IMU, STM board, HC-05 (bluetooth module we did not end up using), and a website for all the pin functionalities on our board. A wiring diagram for Romi can be viewed below, as well as a closeup video of Romis construction:

<img width="843" height="838" alt="WiringDiagram" src="https://github.com/user-attachments/assets/b126c080-8138-42dc-ac5d-6ca469e84e5d" />

[![Video Title](https://img.youtube.com/vi/7AZLOIbpHas/0.jpg)](https://www.youtube.com/watch?v=7AZLOIbpHas)

Below is our complete hardware list, links are included below where applicable for specific parts. View the video above for exact assembly, in order to get the line sensor to the ideal 5mm distance from the ground we had to add a few nuts between the standoff and the top of the line sensor. A nut was also added right below the head of the screw to prevent the screws from being to close to the ground. 

Complete Hardware List:     <br>

4	M2.5 x 8mm Standoff        <br>                                                                                                
4	M2.5 x 10mm Standoff        <br>                                                                                               
4	M2.5 x 30mm Standoff          <br>                                                                                             
4	M2.5 x 6mm Socket Head Cap Screw    <br>                                                                                       
4	M2.5 x 8mm Socket Head Cap Screw   <br>                                                                                        
4	M2.5 x 10mm Socket Head Cap Screw    <br>                                                                                     
8	M2.5 Nylon Lock Nuts    <br>                                                                                                   
8	M2.5 Nylon Washer     <br>                                                                                                     
1	Acrylic Romi-to-Shoe Adapter (convenient but not necessary, custom part)         <br>                                          
1	Modified Shoe of Brian (w/o Bead or Resistors) (documentation for shoe of brian can be located online)  <br>                   
1	Nucleo L476RG    <br>                                                                                                          
1	USB-A to USB-Mini B Cable <br> 

Dupont Style Cables <br>

Line sensor: https://www.pololu.com/product/4245 <br>                                                                         
Nylon Spacers for Line Sensor Mounting: https://www.pololu.com/product/1976  <br>                                                
Screws for Line Sensor Mounting: https://www.pololu.com/product/1958   <br>                                                      
Nuts for Line Sensor Mounting: https://www.pololu.com/product/1067  <br>  
Bump Switch: https://www.pololu.com/product/1405  <br>                                                                           
Motor Driver and Power Distriution Board: https://www.pololu.com/product/3543 <br>                                               
Encoder Pair Kit: https://www.pololu.com/product/3542  <br>                                                                      
Romi Chassis Kit: https://www.pololu.com/product/3504  <br>                                                                      
Rechargable Batteries: https://www.amazon.com/dp/B0CFWXSKDV?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1 <br>                          
HC-05 Blutooth Module (Unused): https://www.amazon.com/dp/B01MQKX7VP   <br>                                                      
Adafruit BNO055 Absolute Orientation Sensor: https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview <br> 

To help Romi better navigate its environment, we wanted to have it be able to locate itself in X Y space as well as recognize its heading. To achieve this we modeled Romi's dynamics and generated a state space model which can be viewed below. This state space model was converted into a discretized state observer to predict Romi's future state. We then could use this future state to assist in closed loop control on Romi's position as it moved around the track.
<img width="430" height="638" alt="image" src="https://github.com/user-attachments/assets/25a26feb-0f04-480e-8c7a-9bd3322af2ec" />

# Code architecture and Task Design Choices

task_generate_curve and task_follow_path

In an attempt to create a very robust method of navigation when not following a line, we created a cubic spline curve generator and an accompanying task that recalculates ROMIs motor setpoints and target trajectory periodically. The curve generator uses parametrically defined points in x and y to compute coefficients a, b, c and d. These coefficients are fed the follow path task, which implements feed forward and feedback control to update motor setpoint values. We were unable to get these tasks to work reliably, and fell back to using a "point and shoot" pose controller to navigate planned trajectories. Our parametrically defined points were graphed and split up using desmos, a free online graphing software to help better visualize the task at hand. A link to our desmos graph is included here: https://www.desmos.com/calculator/alpub6qhm4

task_pose_control

The pose controller task is the working, albeit less robust successor to the feed forward spline generator tasks. This task intakes locally defined x and y coordinates and computes a target heading. It then runs proportional control in order to drive ROMI toward the desired location. When ROMI gets within a desired tolerance, it then orients itself to a desired final heading position, described relative to the initial heading. The pose controller uses the estimated x and y position of ROMI output by our state estimator as the current position.

task_follow_line

Task follow line uses proportional control to steer ROMI toward the centroid of a line by computing changes to a motor setpoint. The line sensor calculates a the centroid relative to the 5 element reflectance sensor array.

task_crash and task_recover

Task crash uses an external interrupt to detect when ROMI has hit an object. This interrupt sets a flag that causes task_recovery to enter an active state. Task recovery causes ROMI to back up and then rotate until it has found a line to follow. When a line has been found, task recovery sets a flag indicating it has completed and resets its internal state.

Final_Path

The Final Path task contains the logic needed to navigate the final obstacle course. Upon meeting certain conditions it transitions states to move to the next movement sequence until the course has been completed. Upon completion Final Task resets its internal state and variables and waits for a user button press to start running the course again.

Located below is our multitasking structure for our final deliverable codebase. It includes all Shares between tasks as dotted lines and all Queues as solid lines. Arrows into or away from the tasks indicate whether the task reads or writes the Queue/Share. It is possible for a task to both read and write a Queue/Share. Queues and Shares are labeled at each task for ease of viewing. This is most definitely not the most efficient way to write the functionality we desired but it worked to our satisfaction. We opted for an approach of making many smaller tasks to perform specific duties instead of large blanket tasks. This was intended to make tasks run quicker and prevent overlap in the scheduler.

<img width="1172" height="2108" alt="TaskDiagram drawio" src="https://github.com/user-attachments/assets/017db014-2a1c-4cff-b558-9fae765ea329" />

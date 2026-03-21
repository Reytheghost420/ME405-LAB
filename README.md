# ME 405 Romi Project
## Final Code Location

The final working implementation of the Romi robot can be found in:

**`Final Demonstration/Final Version/`**

This folder contains the complete and tested code used for the final project demonstration.
## Code
### Drivers

## How to Run the Code

1. Upload all files in this folder to the Nucleo board
2. Ensure `main.py` is present on the device
3. Power the robot and place it on the track
4. The robot will begin execution automatically

Optional:
- Adjust calibration values in `calibration.py`
- Tune control parameters in `task_course.py`
  
#### Motor Driver
This driver sets up the pin objects for the sleep and direction pins and a timer object for the PWM pin.

#### Line Sensor Driver
Our driver reads the 16 bit values of the pins on each channel and assigns each a position value based on the channel's distance away from the center of the sensor. From left to right, this looks like (-3, -2, -1, 0, 1, 2, 3). The raw 16 bit value is normalized between 0 and 1 based on previous calibration data of black and white track surfaces. The normalized sensor values are multiplied with the respective position values. This product is then added up and then divided by the sum of all normalized sensor values to give an error that tells the robot where it is in relation to the center of the line.

Inside this driver, a PI controller takes this error value and outputs shares to the motor task based on selected Kp and Ki values specific to the line sensor driver.

Our line sensor also does the job of detecting checkpoints on the game track by recording when 3 sensors detect a value for black instead of the typical 2 for the line.

### Game Track
<img width="5139" height="2554" alt="Game_Track" src="https://github.com/user-attachments/assets/2d694fcd-8b26-4c78-93bf-3edc1c199b14" />

Our Romi uses its line sensor to guide its motion starting at Checkpoint 0. It then detects it has reached Checkpoint 1 with the line sensor and continues using the line sensor for the arc after #1.
It then turns right 90 degrees based on heading data from the IMU.

After this, our ultrasonic sensor is turned on while the robot drives straight towards the wall. Once the sensor reaches the predetermined distance of 11cm from the wall, the robot initiates its turn left while turning on the line sensor to look for the line. Once the line is detected, the robot moves straight on the line until both the cross and checkpoint #2 are detected.

Then, the robot turns right 90 degrees to orient itself on the slalom line. The line sensor is turned on and that guides the robot until checkpoints #3 and #4 are detected.

Our team opted not to try and knock the cups out of place for the time bonus in order to dedicate our time to completing the track reliably.

#### Video Demonstrations

##### Working Test Course
https://github.com/user-attachments/assets/57b410ec-e20e-4d8b-ae99-a271f1de9f0f

Full Resolution Link: https://youtu.be/C_g4VvyhMvY


##### In Class Demonstration


https://github.com/user-attachments/assets/51b756b1-f9b8-4bad-b1de-43f78a5d53f9

Full Resolution Link: https://youtu.be/82ImlZlA_gw

### Sensors
#### Infrared Sensor
Our sensor is the Pololu QTRX analog sensor. It works by emitting infrared light at the ground and detecting how much light it receives back. At 8mm wide and containing 7 channels, we found it adequate to detect the line and the adjacent white space.

The pins used on our sensor are the VCC, GND, and pins 1, 3, 5, 7, 9, 11, and 13. We soldered right angle male-male pin headers to the sensor to allow for easy connecting to our board.

Link: https://www.pololu.com/product/4447

<img width="600" height="480" alt="image" src="https://github.com/user-attachments/assets/e28aa20e-13dd-4136-b484-7a4b8ed291f7" />

#### Ultrasonic Sensor
Our sensor is the HC-SR04, a common sensor included in many electronics kits. It sends out a high frequency noise and records how long it takes to receive an echo from the noise. The sensor is mounted slightly behind the front of the robot, since accuracy is greatly reduced at distances of under 2cm.

<img width="591" height="396" alt="image" src="https://github.com/user-attachments/assets/ec2562e6-3d96-42e8-8285-f65d674b5d3d" />



### Wiring Diagram
This diagram displays the connections between our Nucleo board, motors, encoders, IMU, IR sensor, and Ultrasonic Sensor

<img width="2050" height="1387" alt="image" src="https://github.com/user-attachments/assets/76dbdf61-9431-47ba-ae36-f1c6fa606702" />


[Romi Wiring Diagram.pdf](https://github.com/user-attachments/files/26132117/Romi.Wiring.Diagram.pdf)

### 3D Printed Parts
#### Infrared Sensor Mount
This mount was created to secure our infrared sensor to the Romi chassis. It keeps the face of the sensor 7cm above the track, which is within the optimal range for the sensor.
2 #2 screws and 2 #2 nuts are used to attach the mount to the chassis and 2 #2 screws attach the sensor to the mount. Those holes in the mount are tapped by the screws themselves and do not require nuts.

<img width="1875" height="1214" alt="image" src="https://github.com/user-attachments/assets/8b99f0ff-e6dc-48d4-9bfd-d9779aafd3cf" />


[Infrared Sensor Mount v1.pdf](https://github.com/user-attachments/files/26132131/Infrared.Sensor.Mount.v1.pdf)


#### Ultrasonic Sensor Mount
This mount was created to attach our ultrasonic sensor to the Romi chassis. It is attached to the chassis with 4 M2.5 screws and nuts and attached to the ultrasonic sensor with hot glue.
This is due to us not having small enough screws to fit through the holes on the ultrasonic sensor.
The mount was originally intended to connect to the chassis on both sides of the suspension ball, but the IMU sensor occupies the chassis space that the ultrasonic sensor mount would otherwise take. Since the ultrasonic sensor is very light, this is not a problem.

<img width="1930" height="1191" alt="image" src="https://github.com/user-attachments/assets/205a4c0a-df78-4aaa-9a15-daf4fb42f9a1" />


Both mounts were created in Fusion 360 using pubilcly available CAD models of the Romi chassis, infrared sensor, and ultrasonic sensor and then 3D printed out of PLA at the makerspace in Mustang 60.

[Ultrasonic Sensor Mount Drawing v1.pdf](https://github.com/user-attachments/files/26132134/Ultrasonic.Sensor.Mount.Drawing.v1.pdf)

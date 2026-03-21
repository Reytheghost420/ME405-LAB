# ME 405 Romi Project
![IMG_7961](https://github.com/user-attachments/assets/eb505301-c711-4a1e-8665-e76e7620c3a9)


## Final Code Location

The final working implementation of the Romi robot can be found in:

**`Final Demonstration/Final Version/`**

This folder contains the complete and tested code used for the final project demonstration.

## Reproducing This Project

To recreate this system, the following hardware components were used:

### Core Hardware
- **Pololu Romi Chassis Kit** (with 6V DC motors and integrated encoders)
- **ST Nucleo-L476RG Microcontroller** (running MicroPython)
- **DRV8838 Motor Drivers** (for left and right motor control)

### Sensors
- **Pololu QTRX-7RC Reflectance Sensor Array (Part #4410 / similar QTRX series)**  
  Used for line detection and position estimation

- **HC-SR04 Ultrasonic Sensor**  
  Used for wall detection in the garage section

- **BNO055 IMU (Bosch Absolute Orientation Sensor)
  Used for heading-based turns (e.g., 90° rotations)

### Additional Components
- Custom **3D printed mounts** for IR and ultrasonic sensors
- Jumper wires and header pins for connections
- External power source (6 1.2V NiMH rechargeable batteries)
- Various screws for attaching mounts to the chassis

---

### Software Setup
1. Install **MicroPython** on the Nucleo-L476RG board  
2. Upload all files from the `Final Demonstration/Final Version/` folder to the board
3. Ensure `main.py` is present on the device and runs on startup

---

### Calibration
- Run `calibration.py` to collect coefficients for the IMU
- Calibration values are stored in `calibration.txt`
- Proper calibration is required for reliable accelerometer, gyroscope, and magnetometer performance in the IMU.

---

### Operation
1. Place the robot at the start of the course (Checkpoint 0)
2. Power the robot
3. The robot will begin execution automatically and follow the programmed course

---

### Notes
- Performance may vary depending on surface conditions, lighting, and battery level
- Fine tuning of parameters in `task_course.py` may be required for optimal performance
  
## Code

## How to Run the Code

1. Upload all files in this folder to the Nucleo board
2. Ensure `main.py` is present on the device
3. Power the robot and place it on the track
4. The robot will begin execution automatically

Optional:
- Adjust calibration values in `calibration.py`
- Tune control parameters in `task_course.py`

### Drivers

#### Motor Driver
This driver sets up the pin objects for the sleep and direction pins and a timer object for the PWM pin. The set effort section controls the direction of the motor and the pulse width percent of the PWM signal.

#### Line Sensor Driver
Our driver reads the 16 bit values of the pins on each channel and assigns each a position value based on the channel's distance away from the center of the sensor. From left to right, this looks like (-3, -2, -1, 0, 1, 2, 3). The raw 16 bit value is normalized between 0 and 1 based on previous calibration data of black and white track surfaces. The normalized sensor values are multiplied with the respective position values. This product is then added up and then divided by the sum of all normalized sensor values to give an error that tells the robot where it is in relation to the center of the line.

Inside this driver, a PI controller takes this error value and outputs shares to the motor task based on selected Kp and Ki values specific to the line sensor driver.

Our line sensor also does the job of detecting checkpoints on the game track by recording when 3 sensors detect a value for black instead of the typical 2 for the line.

### Game Track
<img width="5139" height="2554" alt="Game_Track" src="https://github.com/user-attachments/assets/2d694fcd-8b26-4c78-93bf-3edc1c199b14" />

Our Romi uses its line sensor to guide its motion starting at Checkpoint 0. At startup, the robot enters a line-following state where it continuously reads values from the infrared sensor array. These values are normalized using previously collected calibration data and converted into a position error relative to the center of the line. A proportional-integral (PI) control scheme is then used to adjust the left and right motor speeds, allowing the robot to remain centered on the line while moving forward.

As the robot progresses along the initial straight path, it monitors the sensor readings for a checkpoint condition. Checkpoint 1 is detected when multiple sensors simultaneously register the darker region associated with the checkpoint marker. Upon detecting CP#1, the robot remains in line-following mode but begins navigating the curved arc section of the track. The continuous feedback from the line sensor allows the robot to smoothly follow the curvature without requiring a predefined path.

After completing the arc, the robot transitions to a turning state. At this point, the line sensor is temporarily deprioritized and the robot uses set efforts and time based operations to make a 90 segree turn before driving straight for the wall. 

Once aligned with the next section, the robot enters the garage approach state. In this state, the ultrasonic sensor is activated and the robot drives forward in a controlled manner toward the wall. The ultrasonic sensor continuously measures the distance to the wall, and when the measured distance reaches approximately 11 cm, the robot initiates a left turn. This turn is performed while re-enabling the line sensor so that the robot can search for and reacquire the line during the maneuver.

After the line is detected again, the robot transitions back into line-following mode. As it moves forward, it encounters a cross feature on the track. This feature is detected when several adjacent sensors simultaneously read black, indicating a wide line region rather than a standard narrow line. This condition is used to trigger the next transition. Shortly after the cross, Checkpoint 2 is detected using a similar sensor-based threshold.

Following CP#2, the robot performs another right 90-degree turn using IMU heading feedback to align itself with the slalom section of the course. Once aligned, the robot re-enters line-following mode and navigates the curved slalom path using continuous sensor feedback. The oscillating path requires constant correction, and the PI controller adjusts motor speeds dynamically to maintain stability. The robot continues along the slalom until Checkpoints 3 and 4 are detected.

During this section, the robot also monitors for a loss-of-line condition. If all sensors detect white (indicating that there is a break in the line), the robot enters the post-checkpoint #4 state. In this state, the robot performs a controlled 180-degree turn using IMU feedback to reorient itself. After completing the turn, it resumes searching for the line and continues along the correct path toward the starting checkpoint. 

All of these behaviors are coordinated using a finite state machine implemented in `task_course.py`. Each section of the course corresponds to a specific state, such as line following, turning, wall approach, or line reacquisition. Transitions between states are triggered using a combination of sensor thresholds, IMU heading data, and timing or distance-based conditions.

It is important to note that our team chose not to attempt the cup interaction in order to reduce variability and ensure successful completion of the primary course.

#### Video Demonstrations

##### Working Test Course
https://github.com/user-attachments/assets/57b410ec-e20e-4d8b-ae99-a271f1de9f0f

Full Resolution Link: https://youtu.be/C_g4VvyhMvY

## Final Performance

The robot successfully completed the full course outside of the final demonstration, including:
- straight line tracking
- checkpoint detection
- arc navigation after CP#1
- garage approach and wall detection
- line reacquisition after the wall
- cross detection and continued tracking
- slalom navigation

We were not able to get the robot to consistently complete the course in time for the final demonstration, as errors would sometimes arise that derailed our runs. For all 3 in class runs, the robot turned left after reversing out of checkpoint #4 when it should have gone right.

##### In Class Demonstration


https://github.com/user-attachments/assets/51b756b1-f9b8-4bad-b1de-43f78a5d53f9

Full Resolution Link: https://youtu.be/82ImlZlA_gw

## Known Issues and Improvements

While the robot successfully completes the course, some behaviors may require additional tuning:

- During the garage section, the robot may drift slightly due to small differences in motor speed. This can be corrected by adjusting motor setpoints in `task_course.py`.
- The robot may occasionally begin turning at the middle of the cross instead of at the tip. This is due to early state transitions based on sensor readings. Adjusting thresholds or adding encoder-based checks can improve this behavior.
- Performance may vary depending on surface conditions, lighting, and battery level.

These issues are primarily tuning-related and can be improved without changing the overall system design.

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

## Conclusion

This project demonstrates the successful integration of embedded systems, sensor feedback, and control algorithms to achieve reliable autonomous navigation. By combining line sensing, encoder feedback, and structured state-machine logic, the Romi robot is able to complete a complex course with minimal user input.

Throughout development, emphasis was placed on modular design and iterative testing. Separating motor control, sensor processing, and high-level navigation into distinct tasks allowed for easier debugging and tuning, ultimately leading to a more robust system.

While minor tuning improvements can still be made, the final implementation meets the project objectives and can complete the course, albeit not perfectly consistenctly. This project highlights the importance of both software structure and real-world testing in developing reliable autonomous systems.

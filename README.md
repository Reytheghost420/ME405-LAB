# ME 405 Romi Project
## Project Information
### Code

### Game Track
<img width="5139" height="2554" alt="Game_Track" src="https://github.com/user-attachments/assets/2d694fcd-8b26-4c78-93bf-3edc1c199b14" />

Our Romi uses its line sensor to guide its motion starting at Checkpoint 0. It then detects it has reached Checkpoint 1 with the line sensor and continues using the line sensor for the arc after #1.
It then turns right 90 degrees based on heading data from the IMU.

After this, our ultrasonic sensor is turned on while the robot drives straight towards the wall. Once the sensor reaches the predetermined distance of 11cm from the wall, the robot initiates its 90 degree turn left.


### Wiring Diagram
This diagram displays the connections between our Nucleo board, motors, encoders, IMU, IR sensor, and Ultrasonic Sensor

[Romi Wiring Diagram.pdf](https://github.com/user-attachments/files/26132117/Romi.Wiring.Diagram.pdf)

### 3D Printed Parts
#### Infrared Sensor Mount
This mount was created to secure our infrared sensor to the Romi chassis. It keeps the face of the sensor 7cm above the track, which is within the optimal range for the sensor.
2 #2 screws and 2 #2 nuts are used to attach the mount to the chassis and 2 #2 screws attach the sensor to the mount. Those holes in the mount are tapped by the screws themselves and do not require nuts.
Our sensor is 8mm wide and has 7 channels, which we found adequate to detect the line and the adjacent white space.

Link: https://www.pololu.com/product/4447

[Infrared Sensor Mount v1.pdf](https://github.com/user-attachments/files/26132131/Infrared.Sensor.Mount.v1.pdf)


#### Ultrasonic Sensor Mount
This mount was created to attach our ultrasonic sensor to the Romi chassis. It is attached to the chassis with 4 M2.5 screws and nuts and attached to the ultrasonic sensor with hot glue.
This is due to us not having small enough screws to fit through the holes on the ultrasonic sensor.
The mount was originally intended to connect to the chassis on both sides of the suspension ball, but the IMU sensor occupies the chassis space that the ultrasonic 
sensor mount would otherwise take.

[Ultrasonic Sensor Mount Drawing v1.pdf](https://github.com/user-attachments/files/26132134/Ultrasonic.Sensor.Mount.Drawing.v1.pdf)

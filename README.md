# Autonomous-Car
### Developed an autonomous car using a Robot Operating System(ROS) and implemented control algorithms for lateral and longitudinal control applying locomotion, localization, and navigation 1:18 Ackerman-based RC car.

#### This project has been a remarkable journey of learning and growth. In this project, my teammates and I worked on the locomotion, localization, navigation, and control of a 1:18 Ackerman-based RC car using ROS (Robot Operating System). The car was actuated using a rear DC motor and steered using a front Servo motor.

#### For the software, we utilized the Gazebo simulation environment on Ubuntu 20.04, implementing and testing various nodes and communication protocols. In the embedded hardware, we integrated two sensors for localization and navigation, a magnetic encoder for speed and odometry state measurement, and an IMU for heading state measurement. These sensors were connected to the Raspberry Pi 4, which served as the embedded processor responsible for implementing the control.

#### To control the car, we employed a PID algorithm for longitudinal control. Additionally, we tested two different algorithms for lateral control: Stanley and Pure-pursuit. The ultimate goal of the project was to develop a teleoperation node, allowing us to control the car from a computer's keyboard arrows, make it move in a 10m straight line while keeping its lane, and finally, enable it to avoid obstacles by shifting lanes.

## Youtube Video:
[![Watch the video](https://github.com/ahmadmadyy/Autonomous-Car/assets/98853949/fef6c554-081d-4262-9766-0ec8184e87d7)](https://www.youtube.com/watch?v=9ybnkhmj1Nw)

[<img src="https://github.com/ahmadmadyy/Autonomous-Car/assets/98853949/fef6c554-081d-4262-9766-0ec8184e87d7" width="600" height="300"
/>](https://www.youtube.com/watch?v=9ybnkhmj1Nw)



# NXPCUP COMPETITION

Programed a microcontroller to maneuver a car autonomously around a track, building firmware with PX4 and optimizing software for time complexity

## Autonomous Vehicle Control

I implemented a geometric path-tracking controller based on the **Stanley** method to enable precise autonomous maneuvering. The controller computes two key error metrics:

- **Heading Error**: The angular difference between the car's current orientation and the desired path direction
- **Cross-Track Error**: The lateral displacement between the car's position and the reference path

By dynamically adjusting steering based on these errors, the controller ensures **path accuracy** and **adaptive turning**. Created a simple linear equation for the vehicle's speed as a function of its steering, where both values should be calculated and returned within the interval [-1, +1]


For more information about the firmware, please check the official website for NXPCUP
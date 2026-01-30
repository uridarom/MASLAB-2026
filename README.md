Code for the Team 8 robot that competed in the MIT Mobile Autonomous Systems Laboratory (MASLAB) competition in 2026. 
The robot came 2nd place in the competition.

The objective of the robot is to collect cylindrical-shaped cans of different colors (red, green or yellow) and deposit them in their respective goals. 
At the same time, another robot is competing to place the same cans in its own set of goals. 
Both robots have 2.5 minutes to gather as many points as they can (10 points if a can is in its appropriate goal, 5 points if it is in a different goal).

This program built a full 2D map of the robot's environment as it played. It used odometry to determine the robot's position, combined with homography 
using a camera in order to determine the real-world coordinates of various objects. It then used this information to build a map that allowed for object permanence,
meaning the robot could go to specific features in the map (such as cans and goals) without needing them to have them in the camera's field of view.

The robot featured a sweeping mechanism with surgical tubing used as the intake mechanism. This allowed it to carry up to two cans at once and deposit them precisely.

The UI, featuring the 2D map:

<img width="1239" height="700" alt="Screenshot 2026-01-30 at 3 15 35 PM" src="https://github.com/user-attachments/assets/973d51b7-d477-4c2d-be3c-461339e54825" />

The robot:

![IMG_7826](https://github.com/user-attachments/assets/1dcda936-3ee9-46f8-8b31-9d1a316299a9)

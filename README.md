# Intelligent-Robots
Repository for Turtlebot 3 autonomous navigation from scratch in Python. Find scripts in ***src/beginner_tutorials/*** package for localization, computer vision, proportional-integral-derivative controllers, and path planning. The implementation of all nodes will grant Turtlebot the capability to play soccer.

## Nodes
***prison_break.py*** - This node provides an event loop that responds to the robots environment. If the robot bumps into an object of its environment, the event loop will respond appropriately by letting the robot back up, turn, and continue on its path.

***ball_detector.py*** - This node provides Turtlebot with the capability to detect certain objects in its environment. The node will publish bearing and distance messages if there is a certain object that is in the robot's field of view

***robot_puppy*** - This node provides another event loop which will use the bearing and distance messages published by the ***ball_detector.py*** node. This node implements a PID controller for approaching desired destinations in its environment. This node also implements a path planning strategy, which uses a vector function to return the euclidean distance and polar theta to be passed into PID controllers to follow a path to a desired arbitrary location published by the odometry topic.

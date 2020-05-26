# ackermann_steering
- Image of world model Ackermann Steering.
![Model](image/model.png)

* USAGES *

- roscore
- roslaunch my_robot gazebo_launch.launch 				\t#Turn on gazebo simulation
- roslaunch my_robot controller.launch	 				\t#Turn on controller for simulation
- roslaunch obstacle_detector my_launch.launch 				\t#Turn on obstacle detector, move robot to a position, that can detector object.
- rosrun my_robot move_base.py						\t#Move to started point of tracjectory.
- rosrun my_robot move_slalom.py					\t#Move slalom trajectory.

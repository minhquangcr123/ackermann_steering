# ackermann_steering
## Setup work flow
### Open terminal and mkdir 1 dictionary for project (installed virtualenv first)
```
mkdir -p project_ackermann
cd /project_ackermann
virtualenv project_env #virtual enviroment
cd /project_env
source ./bin/activate #activate enviroment
git clone https://github.com/minhquangcr123/ackermann_steering
pip install -r requirement # install package, dependiences. 
cd #Back to root direction
gedit .bashrc # Open file .bashrc
```
coppy three line to .bashrc file and save

```
source ~/catkin_ws/devel/setup.bash 
export GAZEBO_MODEL_PATH= ~/model_building
export GAZEBO_RESOURCE_PATH= ~/robot_ws/src/my_robot/src
```
Then go to catkin workspace and run makefile
```
cd project_ackermann/project_env/robot_ws
catkin_workspace
```
## Usages
- roscore
- roslaunch my_robot gazebo_launch.launch 				(Turn on gazebo simulation)
- roslaunch my_robot controller.launch	 				(Turn on controller for simulation)
- roslaunch obstacle_detector my_launch.launch 				(Turn on obstacle detector, then move robot to a position that can detector object)
- rosrun my_robot move_base.py						(Move to started point of tracjectory.)
- rosrun my_robot move_slalom.py					(Move slalom trajectory.)

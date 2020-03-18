# Udacity_Project_2-Go_Chase_IT
##  Introduction
In this second project for the Udacity Robotics NanoDegree, the task at hand was to design a robot and program it to chase white-colored balls.

## Setup
There are two packages within the repository:
⋅⋅* my_robot
..* ball_chaser

To run the code, first create a catkin workspace:
``` bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```
Then clone the repository into src, and build the workspace (make sure you're in the master branch): 
``` bash
git clone https://github.com/ThanhL/Udacity_Project_2-Go_Chase_IT.git
cd ..
catkin_make
```
Now launch the world with the robot:
``` bash
source devel/setup.bash
roslaunch my_robot world.launch
```

And launch the ball_chaser node:
``` bash
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```

You can now move the ball in front of the robot and it will start chasing the white ball.
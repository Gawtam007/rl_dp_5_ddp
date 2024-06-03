# rl_dp_5_ddp

## Autonomous Ultrasound Scan using a Single RGB Camera

### Description
This project 

### Contents 

```
Moveit_Igus_RL_DP5
├── documents
├── igus_examples
```

### Using of the repo

Clone the project in the src folder of your ROS workspace
```
cd path_to_your_ws/src
git clone 
```

Compile the project 
```
catkin build
source devel/setup.bash
```

Visualization

  - To visualize the arm using Moveit and RViz

  ```
  roslaunch rldp5_moveit demo.launch
  ```

  - To apply controllers on the arm using Gazebo and 
  ```
  roslaunch rldp5_moveit demo_gazebo.launch
  ```



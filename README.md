# rl_dp_5_ddp

## Autonomous Ultrasound Scan using a Single RGB Camera

### Description
This project 

### Contents 

```
rl_dp_5_ddp
├── colmap
│     ├── 100_phantom_frames (Set of images/frames obtained from video of phantom via phone)
│     ├── 100_phantom_ws (corresponding colmap workspace generated to obtain the pointcloud)
│     ├── dress_phanton_frames
│     └──  dress_phantom_ws
├── crg_ws
│     ├── rldp5_description (contains the urdf, mesh files, textures, and other necessary files to visualise the arm)
│     └──  rldp5_moveit (moveit package genrated using the moveit_setup_assistant)
└──  rldp5_ws
      ├── ply_publisher (external package used to convert .ply to ros pcl message)
      ├── rldp5_description
      ├── rldp5_moveit 
      └── rldp5_scripts (contains all the .py scripts required)
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

### Simulation

- Once you've built and setup the workspace, launch rviz and gazebo simulation of the arm using the commands above
- To start publishing the processed point cloud
  ```
   roslaunch point_cloud_io human_body_read.launch
  ```
- To segment the point cloud
  ```
  rosrun rldp5_scripts segment_pcl.py
  ```
- To smoothen the point cloud
  ```
  rosrun rldp5_scripts smoothen.py
  ```
- To estimate normals on the point cloud
  ```
  rosrun rldp5_scripts estimate_normals.py
  ```
  
- Additionally
  - waypoints.py can be used to move to arm to specified joint angles
  - frame.py can be used to break a video into frames
  - save_images.py to store the images obtained from the ros camera



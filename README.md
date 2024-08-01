Video: https://youtu.be/fHNMj_VB3kc?si=0IjlrRFa1Jm7FOBZ

### ROS version
ros noetic

### HARDWARE 
rplidar A1M8, OpenManipulator-X

### PURPOSE
2D LaserScan => 3D PointCloud2 following end-effector's state  



![Screenshot from 2024-04-17 11-17-25](https://github.com/sjahn2000/lidar_manipulator/assets/60663351/edaf41c2-5d07-4bfa-8d7a-5a4235ae80db)


### INSTALL 
    $ cd catkin_ws/src  
    $ git clone https://github.com/sjahn2000/lidar_manipulator.git

### BUILD  

    $ cd .. && catkin_make  
    $ source dev/setup.bash

### LAUNCH  

    $ roslaunch lidar_manipulator lidar_manipulator.launch

(But if you want this package to work properly, you should also launch rplidar_ros and open_manipulator_controller. So, refer to the links below)  
https://github.com/Slamtec/rplidar_ros  
https://github.com/ROBOTIS-GIT/open_manipulator

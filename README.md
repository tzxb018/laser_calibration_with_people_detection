Laser Calibration with People Detection
===========
![ROS](https://img.shields.io/badge/ROS-Kinetic-brightgreen.svg)  ![OS](https://img.shields.io/badge/OS-Ubuntu%2016.04-orange.svg )

Overview
------------------
This ROS package combines multiple laser scans from RPLIDAR A3M1 lasers into a calibrated point cloud. This package will then track the ankles of moving people in the point cloud and predict their paths with linear regression. 

**Keywords:** 2-dimensional laser, RPLIDAR A3M1, point cloud, matrix transformations, calibration, linear regression

## Authors

* **Tomohide Bessho** - *UNR REU student* - https://github.com/tzxb018
* **Andrew Palmer** - *UNR Grad Mentor* - https://github.com/ahpalmerUNR

Install
-------------------
- Clone rplidar_ros into your kinetic catkin source directory: https://github.com/Slamtec/rplidar_ros
- Install sckikit-learn: https://scikit-learn.org/stable/install.html
  ```
  pip install -U scikit-learn
  ```
  - If you are using Python 2.7, you will need to install scikit-learn 0.20 instead of the latest version
- Run the following commands to install the package
  ```
	cd catkin_workspace/src
	git clone https://github.com/tzxb018/laser_calibration_with_people_detection/
	cd ../
	catkin_make
  ```
Usage
-------------------
- Use the combined launch file to run the package
  ```
  roslaunch lc start.launch
  ```
- If this launch file does not work (normally happens when playing back a rosbag file), you will need to run each script sepeartely. Do each command in a sepearte terminal.
    ```
    roscore
    rosrun rviz rviz
    rosbag play -l <rosbag.file> (If you are running a rosbag file)
    rosrun lc laser_tf.py
    rosrun lc detection_target.py <len. of leg 1><len. of leg 2><len. of leg 3><radius of target>
    rosrun lc tf_publisher.py
    rosrun lc people_detection.py
    ```
- If this script gives a "tf2.ExtrapolationException: Lookup would require extrapolation into the past." error, rerun the script until the error resolves (only occurs when playing an old rosbag file).  
      
Important Notes
-------------------
- To search for the triangular calibration target, you will need to change the arguments for the detection_target.py script.
    - If you are using the launch file, please go into the launch file and change the arguments in the following format:
      - length of shortest leg, length of longer leg, length of hypotnuse, length of radius of targets (all in meters)
      - If you are manually running each script, add the same agruments to the script command. 
      - Ex: $ rosrun lc detection_target.py .57 .89 1.05 .07
- The laser subscriber topics (explained after), were all namespaced into the following three names: hog, mouse, snake. 
    - If you are trying to use four lasers instead of three, go into the tf_publisher.py script and uncomment lines 156, 179-181, 218-223, and 283-293. 
    - Also, go into people_detection.py script and uncomment lines 395, 404-405, 419, 488-504.
- To launch the lasers, download and catkin_make this package onto each Raspberry Pi with the RPLIDAR A3M1 laser connected to it. For each Pi, run a different launch file. Please make sure that the Pis run lc_hog.launch, lc_mouse.launch, lc_snake.launch, and lc_duck.launch in that following order.

Subscribed Topics
-------------------
- /hog/scan0
  - the parent laser for the trasnformations
- /mouse/scan0
  - one of the children lasers
- /snake/scan0 
  - another one of the children lasers
- /duck/scan0
  - if you are running 4 lasers, this would be another child laser

Published Topics
-------------------
- /ankles
  - displays the ankle markers and the prediction markers on rviz
- /combined
  - displays the overlap of all the lasers being run as a point cloud
- /combined_movement
  - displays the overlap of all the lasers that have detected movement as a point cloud
- /possible/triangles
  - displays the triangular calibration target in rviz
- /tf
  - manifests the tf between the different lasers
  
Under the Hood
-------------------
**Files for running**

- src/laser_tf.py - calculates the tfs between two different lasers
- src/detection_target.py - finds and calibrates to the calibration target by searching for cirlces and finding the distances between each circle to search for the triangular shape of the target. *Requires the lengths of the three legs of the target in ascending order and the radius of the circles of the target as arguments in the terminal*
- src/tf_publisher.py - publishes the calculated tfs and applies them to the laser scans. The publisher publishes the combined scans as a point cloud.
- src/people_detection.py - searches for ankles in the combined point cloud, marks and tracks them, and uses linear regression to predict their paths.

Acknowledgement
-------------------
We used some code from the Palmer's pr2_intent_rec package: https://github.com/ahpalmerUNR/pr2_intent_rec.
The circle detection algorithm is modified from this package to better suit the laser scans we use in this package. 

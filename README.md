Laser Calibration with People Detection
===========

Usage
-------------------
- clone laser_calibration_with_people detection into your kinetic catkin source directory
- clone rplidar_ros into your kinetic catkin source directory: https://github.com/Slamtec/rplidar_ros
- install sckikit-learn: https://scikit-learn.org/stable/install.html
  - $ pip install -U scikit-learn
  - if you are using Python 2.7, you will need to install scikit-learn 0.20 instead of the latest version
- $ cd [your catkin_workspace]
- $ catkin_make
- $ roslaunch lc start.launch
  - if this launch file does not work (normally happens when playing back a rosbag file), you will need to run each script sepeartely. Do each command in a sepearte terminal
    - $ roscore
    - $ rosrun rviz rviz
    - $ rosbag play -l <rosbag.file>
    - $ rosrun lc laser_tf.py
    - $ rosrun lc detection_target.py <len. of leg 1 len. of leg 2 len. of leg 3 radius of target>
    - $ rosrun lc tf_publisher.py
    - $ rosrun lc people_detection.py
      - if this script gives a "tf2.ExtrapolationException: Lookup would require extrapolation into the past." error, rerun the script until the error resolves (only occurs when playing an old rosbag file).  

Subscribed topics
-------------------
- /hog/scan0
  - the parent laser for the trasnformations
- /mouse/scan0
  - one of the children lasers
- /snake/scan0 
  - another one of the children lasers


Published topics
-------------------
- /ankles
  - displays the ankle markers and the prediction markers on rviz
- /combined
  - displays the overlap of all the lasers being run as a point cloud
- /combined_movement
  - displays the overlap of all the lasers that have detected movement as a point cloud
- /tf
  - manifests the tf between the different lasers

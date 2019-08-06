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



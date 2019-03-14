#Vins-YOLO 
   We combine a CNN detector with VINS-Mono, a moving visual odometry system, and show reliable improvement in the detection process, especially as the robot starts to accelerate. Our system is ready-to-use in that it has very low deployment cost and requires no calibration. The resulting system allows for simultaneous robot state estimation and object detection, as well as object tracking. Lastly, the architecture is flexible in that it is not restrained to a specific object type or detector. 

# VINS-Mono 
VINS-Mono is a real-time SLAM framework for **Monocular Visual-Inertial Systems**. It uses an optimization-based sliding window formulation for providing high-accuracy visual-inertial odometry. It features efficient IMU pre-integration with bias correction, automatic estimator initialization, online extrinsic calibration, failure detection and recovery, loop detection, and global pose graph optimization, map merge, pose graph reuse, online temporal calibration, rolling shutter support. VINS-Mono is primarily designed for state estimation and feedback control of autonomous drones, but it is also capable of providing accurate localization for AR applications. This code runs on **Linux**, and is fully integrated with **ROS**. For **iOS** mobile implementation, please go to [VINS-Mobile](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile).

**Authors:** [Tong Qin](http://www.qintonguav.com), [Peiliang Li](https://github.com/PeiliangLi), [Zhenfei Yang](https://github.com/dvorak0), and [Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [HUKST Aerial Robotics Group](http://uav.ust.hk/)

# YOLO
YOLO is a real time object detection algorithm.
**Authors:** 
Joseph Redmon, Santosh Divvala, Ross Girshick, Ali Farhadi

##  Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong QIN <tong.qinATconnect.ust.hk> or Peiliang LI <pliapATconnect.ust.hk>.

For commercial inquiries, please contact Shaojie SHEN <eeshaojieATust.hk>

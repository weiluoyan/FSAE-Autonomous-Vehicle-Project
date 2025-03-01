# FSAE-Autonomous-Vehicle-Project

🚀 Formula SAE Autonomous Vehicle Development
📌 Project Overview

This project focuses on developing an Autonomous Vehicle (AV) system for Formula SAE (FSAE) competitions using ROS, LiDAR, SLAM, and CARLA Simulator. The goal is to create a self-driving racing car capable of detecting cones, planning paths, and navigating the track autonomously.

🎯 Project Goals
	1.	Perception System: Implement LiDAR & Camera-based object detection for cone recognition.
	2.	Localization & Mapping: Develop SLAM using LiDAR and IMU data to build an accurate track map.
	3.	Path Planning & Control: Implement A / RRT* algorithms for autonomous navigation.
	4.	Simulation & Testing: Utilize CARLA and Gazebo for virtual testing before real-world implementation.
	5.	Hardware Integration: Deploy AV software on Hesai Pandar40 LiDAR and Jetson TX2.

 📦 Tech Stack & Dependencies
	•	Programming Languages: Python, C++
	•	Frameworks: ROS2 (Robot Operating System)
	•	Simulation Tools: CARLA, Gazebo
	•	Perception: OpenCV, PCL (Point Cloud Library)
	•	Algorithms: SLAM (Cartographer, LOAM), Path Planning (A*, RRT)
	•	Hardware: Hesai Pandar40 LiDAR, Jetson TX2

 ⚙️ Setup & Installation

1️⃣ Install ROS & Dependencies
sudo apt update && sudo apt upgrade -y
sudo apt install ros-foxy-desktop python3-colcon-common-extensions

2️⃣ Install CARLA Simulator
git clone https://github.com/carla-simulator/carla.git
cd carla
./Update.sh && ./CarlaUE4.sh -opengl

3️⃣ Clone & Build the Project
git clone https://github.com/weiluoyan/FSAE-Autonomous-Vehicle-Project.git
cd FSAE-Autonomous-Vehicle-Project
colcon build --symlink-install

4️⃣ Run the Simulation
source install/setup.bash
roslaunch carla_ros_bridge carla_ros_bridge.launch


📊 Current Progress

✅ Installed ROS2, CARLA, and Gazebo
✅ Integrated Hesai Pandar40 LiDAR with ROS
✅ Implemented DBSCAN-based LiDAR cone detection
✅ Developed CARLA-based AV simulations
🔄 Next Steps: Implement SLAM & real-time path planning

⭐ Support

If you find this project helpful, please give it a star ⭐ on GitHub! 🚀

#!/usr/bin/env python3
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
import open3d as o3d

# ROS publisher for detected cones visualization
cone_marker_pub = rospy.Publisher('/detected_cones', MarkerArray, queue_size=10)

# **Updated Cone Dimensions (meters)**
CONE_HEIGHT_RANGE = (0.05, 0.12)  # Height range (90mm -> 0.09m)
CONE_BOTTOM_WIDTH_RANGE = (0.10, 0.28)  # Base width (200mm -> 0.2m)
CONE_TOP_WIDTH_RANGE = (0.020, 0.05)  # Top width (40mm -> 0.04m)
CONE_HEIGHT_TO_BASE_RATIO = (0.3, 0.8)  # Expected ratio for stability

# **DBSCAN Clustering Parameters**
DBSCAN_EPS = 0.1  # Reduce eps to ensure small cones are correctly clustered
DBSCAN_MIN_SAMPLES = 2  # Adjust for smaller cones

# **ROI Filtering (Region of Interest)**
ROI_X_MIN, ROI_X_MAX = -1.0, 5.0  # Forward detection range
ROI_Y_MIN, ROI_Y_MAX = -3.0, 3.0  # Side detection range
ROI_Z_MIN = -0.02  # Min height threshold (avoid ground noise)

# **LiDAR Position (your measurements)**
LIDAR_HEIGHT = 0.35  

# ----------------------------
# **Step 1: Ground Removal**
# ----------------------------
def remove_ground(lidar_points):
	"""Removes ground points using RANSAC plane segmentation."""
	rospy.loginfo("Starting ground removal...")
    
	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(lidar_points)

	plane_model, inliers = pcd.segment_plane(distance_threshold=0.008, ransac_n=3, num_iterations=1000)
	non_ground = pcd.select_by_index(inliers, invert=True)
    
	rospy.loginfo(f"Ground removal complete. Kept {len(non_ground.points)} non-ground points.")
	return np.asarray(non_ground.points)

# ----------------------------
# **Step 2: Shape Validation**
# ----------------------------
def is_cone_shape(cluster_points):
	"""Determines if a clustered object has a cone-like shape based on real dimensions."""
	min_z = np.min(cluster_points[:, 2])
	max_z = np.max(cluster_points[:, 2])
	height = max_z - min_z  # Compute height

	base_width_x = np.ptp(cluster_points[:, 0])  # X width range
	base_width_y = np.ptp(cluster_points[:, 1])  # Y width range
	base_width = max(base_width_x, base_width_y)

	# Compute top width using the 90th and 10th percentile for robustness
	top_width_x = np.percentile(cluster_points[:, 0], 90) - np.percentile(cluster_points[:, 0], 10)
	top_width_y = np.percentile(cluster_points[:, 1], 90) - np.percentile(cluster_points[:, 1], 10)
	top_width = max(top_width_x, top_width_y)

	rospy.loginfo(f"Cluster Stats - Height: {height:.3f}m, Base: {base_width:.3f}m, Top: {top_width:.3f}m")

	# Check shape constraints
	if not (CONE_HEIGHT_RANGE[0] <= height <= CONE_HEIGHT_RANGE[1]):
    	rospy.logwarn(f"Rejected: Height {height:.3f}m out of range.")
    	return False
	if not (CONE_BOTTOM_WIDTH_RANGE[0] <= base_width <= CONE_BOTTOM_WIDTH_RANGE[1]):
    	rospy.logwarn(f"Rejected: Base width {base_width:.3f}m out of range.")
    	return False
	if not (CONE_TOP_WIDTH_RANGE[0] <= top_width <= CONE_TOP_WIDTH_RANGE[1]):
    	rospy.logwarn(f"Rejected: Top width {top_width:.3f}m out of range.")
    	return False
	if not (CONE_HEIGHT_TO_BASE_RATIO[0] <= height / base_width <= CONE_HEIGHT_TO_BASE_RATIO[1]):
    	rospy.logwarn(f"Rejected: Height/Base ratio {height/base_width:.2f} out of range.")
    	return False

	return True

# ----------------------------
# **Step 3: Main LiDAR Processing**
# ----------------------------
def process_lidar_data(point_cloud):
	"""Processes LiDAR data to detect cones."""
	rospy.loginfo("Processing LiDAR data...")

	points = np.array([p[:3] for p in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)])
	rospy.loginfo(f"Total LiDAR points: {len(points)}")

	non_ground_points = remove_ground(points)

	roi_mask = (
    	(non_ground_points[:, 0] > ROI_X_MIN) & (non_ground_points[:, 0] < ROI_X_MAX) &
    	(non_ground_points[:, 1] > ROI_Y_MIN) & (non_ground_points[:, 1] < ROI_Y_MAX) &
    	(non_ground_points[:, 2] > ROI_Z_MIN)
	)
	roi_points = non_ground_points[roi_mask]
	rospy.loginfo(f"Filtered {len(roi_points)} points in ROI.")

	clustering = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(roi_points)
	labels = clustering.labels_
	rospy.loginfo(f"Clusters found: {set(labels)}")

	detected_cones = []

	for label in set(labels):
    	if label == -1:
        	continue
    	cluster_pts = roi_points[labels == label]
    	if is_cone_shape(cluster_pts):
        	centroid = np.mean(cluster_pts, axis=0)
        	detected_cones.append(centroid)
        	rospy.loginfo(f"Detected Cone at: X={centroid[0]:.2f}, Y={centroid[1]:.2f}, Z={centroid[2]:.2f}")

	rospy.loginfo(f"Total detected cones: {len(detected_cones)}")
	publish_cones(detected_cones)

# ----------------------------
# **Step 4: Publish Markers to Rviz**
# ----------------------------
def publish_cones(cone_positions):
	"""Publishes detected cones as ROS markers in Rviz."""
	marker_array = MarkerArray()
	for idx, position in enumerate(cone_positions):
    	marker = Marker()
    	marker.header.frame_id = "Pandar40P"
    	marker.header.stamp = rospy.Time.now()
    	marker.ns = "cones"
    	marker.id = idx
    	marker.type = Marker.CYLINDER
    	marker.action = Marker.ADD
    	marker.pose.position.x = position[0]
    	marker.pose.position.y = position[1]
    	marker.pose.position.z = position[2] / 2
    	marker.scale.x = 0.2
    	marker.scale.y = 0.2
    	marker.scale.z = 0.09
    	marker.color.a = 1.0
    	marker.color.r = 1.0
    	marker.color.g = 0.5
    	marker.color.b = 0.0
    	marker_array.markers.append(marker)
	cone_marker_pub.publish(marker_array)

rospy.init_node('cone_detector')
rospy.Subscriber('/hesai/pandar', PointCloud2, process_lidar_data)
rospy.spin()


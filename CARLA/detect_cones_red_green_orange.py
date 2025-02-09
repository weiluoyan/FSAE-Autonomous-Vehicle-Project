#!/usr/bin/env python3
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
from filterpy.kalman import KalmanFilter
import open3d as o3d

# Publisher for detected cones visualization
cone_marker_pub = rospy.Publisher('/detected_cones', MarkerArray, queue_size=10)

# Initialize Kalman Filter for tracking cones
kf = KalmanFilter(dim_x=4, dim_z=2)
kf.F = np.array([[1, 0, 1, 0],  # State transition matrix
             	[0, 1, 0, 1],
             	[0, 0, 1, 0],
             	[0, 0, 0, 1]])
kf.H = np.array([[1, 0, 0, 0],  # Measurement function
             	[0, 1, 0, 0]])
kf.P *= 1000  # Initial uncertainty
kf.R = np.eye(2) * 10  # Measurement noise

def remove_ground(lidar_points):
	"""Remove ground points using RANSAC"""
	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(lidar_points)

	# Apply RANSAC for ground plane segmentation
	plane_model, inliers = pcd.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=1000)
    
	# Extract non-ground points
	non_ground = pcd.select_by_index(inliers, invert=True)
	return np.asarray(non_ground.points)

def process_lidar_data(point_cloud):
	"""Process Hesai LiDAR data and detect cones."""
	# Extract XYZ points from PointCloud2
	points = np.array([p[:3] for p in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)])

	# Remove ground points
	non_ground_points = remove_ground(points)

	# Define Region of Interest (ROI)
	cone_roi = (non_ground_points[:, 0] > -5) & (non_ground_points[:, 0] < 60) & \
           	(non_ground_points[:, 1] > -20) & (non_ground_points[:, 1] < 20)
	cone_points = non_ground_points[cone_roi]

	if cone_points.shape[0] == 0:
    	rospy.loginfo("No cones detected in ROI.")
    	return

	# Apply DBSCAN clustering
	clustering = DBSCAN(eps=1.2, min_samples=3).fit(cone_points)
	labels = clustering.labels_

	# Filter valid clusters
	valid_cones = [label for label in set(labels) if label != -1 and np.sum(labels == label) >= 5]
	detected_cones = []

	for label in valid_cones:
    	cluster_points = cone_points[labels == label]
    	centroid = cluster_points.mean(axis=0)  # Compute cluster centroid
    	detected_cones.append(centroid)

	rospy.loginfo(f"Detected {len(detected_cones)} cones.")
	track_cones(detected_cones)
	publish_cones(detected_cones)

def track_cones(detected_cones):
	"""Track cones using Kalman Filter"""
	global kf
	for cone in detected_cones:
    	kf.predict()
    	kf.update(cone[:2])  # Use only x, y coordinates
    	rospy.loginfo(f"Tracked cone position: {kf.x[:2]}")

def publish_cones(cone_positions):
	"""Publish detected cones as ROS markers"""
	marker_array = MarkerArray()

	for idx, position in enumerate(cone_positions):
    	marker = Marker()
    	marker.header.frame_id = "Pandar40P"
    	marker.header.stamp = rospy.Time.now()
    	marker.ns = "cones"
    	marker.id = idx
    	marker.type = Marker.SPHERE
    	marker.action = Marker.ADD
    	marker.pose.position.x = position[0]
    	marker.pose.position.y = position[1]
    	marker.pose.position.z = position[2]
    	marker.scale.x = 0.5
    	marker.scale.y = 0.5
    	marker.scale.z = 0.5
    	marker.color.a = 1.0  # Marker transparency
    	marker.color.r = 1.0  # Red
    	marker.color.g = 0.5  # Green
    	marker.color.b = 0.0  # Orange color

    	marker_array.markers.append(marker)

	cone_marker_pub.publish(marker_array)

def main():
	"""Main function to initialize ROS and process LiDAR data."""
	rospy.init_node('cone_detector', anonymous=True)
    
	# Subscribe to Hesai Pandar LiDAR topic
	rospy.Subscriber('/hesai/pandar', PointCloud2, process_lidar_data)

	rospy.loginfo("Cone detection node started. Listening for LiDAR data...")
	rospy.spin()

if __name__ == "__main__":
	main()


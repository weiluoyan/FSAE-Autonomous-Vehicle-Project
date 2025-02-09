#!/usr/bin/env python3
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv2
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
from filterpy.kalman import KalmanFilter
import open3d as o3d

# ROS publishers
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

# Initialize OpenCV Bridge
bridge = CvBridge()
latest_image = None  # Store the latest camera image


def remove_ground(lidar_points):
    """Remove ground points using RANSAC"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_points)

    # Apply RANSAC for ground plane segmentation
    _, inliers = pcd.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=1000)
    
    # Extract non-ground points
    non_ground = pcd.select_by_index(inliers, invert=True)
    return np.asarray(non_ground.points)


def process_camera_image(image_msg):
    """Process the camera image to detect blue objects."""
    global latest_image
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

    # Convert to HSV and apply color thresholding for blue
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([90, 50, 50])  # Lower bound for blue in HSV
    upper_blue = np.array([130, 255, 255])  # Upper bound for blue in HSV
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Save the latest processed mask
    latest_image = mask


def is_blue_object(cluster_points, img_coords):
    """Check if a cluster corresponds to a blue object in the camera image."""
    if latest_image is None:
        return False  # No image available yet

    # Check if the corresponding pixels in the image are blue
    for (x, y) in img_coords:
        if 0 <= x < latest_image.shape[1] and 0 <= y < latest_image.shape[0]:  # Bounds check
            if latest_image[y, x] > 0:  # If pixel is blue in the mask
                return True
    return False


def project_lidar_to_camera(lidar_points):
    """Project 3D LiDAR points onto the camera image (simple approximation)."""
    img_coords = []
    for point in lidar_points:
        x, y, z = point[:3]

        # Simple projection (assumes camera is facing forward)
        img_x = int(320 + (x / z) * 300)  # Adjust projection scale
        img_y = int(240 - (y / z) * 300)  # Flip y-axis

        img_coords.append((img_x, img_y))
    return img_coords


def process_lidar_data(point_cloud):
    """Process LiDAR data and detect blue cones."""
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

    detected_cones = []

    for label in set(labels):
        if label == -1:
            continue  # Ignore noise

        cluster_points = cone_points[labels == label]
        img_coords = project_lidar_to_camera(cluster_points)

        # Only detect objects that are confirmed as blue by the camera
        if np.sum(labels == label) >= 5 and is_blue_object(cluster_points, img_coords):
            centroid = cluster_points.mean(axis=0)  # Compute cluster centroid
            detected_cones.append(centroid)

    rospy.loginfo(f"Detected {len(detected_cones)} blue cones.")
    track_cones(detected_cones)
    publish_cones(detected_cones)


def track_cones(detected_cones):
    """Track cones using Kalman Filter"""
    global kf
    for cone in detected_cones:
        kf.predict()
        kf.update(cone[:2])  # Use only x, y coordinates
        rospy.loginfo(f"Tracked blue cone position: {kf.x[:2]}")


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
        marker.color.r = 0.0  # No Red
        marker.color.g = 0.0  # No Green
        marker.color.b = 1.0  # Blue color

        marker_array.markers.append(marker)

    cone_marker_pub.publish(marker_array)


def main():
    """Main function to initialize ROS and process sensor data."""
    rospy.init_node('cone_detector', anonymous=True)

    # Subscribe to Hesai Pandar LiDAR topic
    rospy.Subscriber('/hesai/pandar', PointCloud2, process_lidar_data)

    # Subscribe to camera topic
    rospy.Subscriber('/carla/camera/front/image', Image, process_camera_image)

    rospy.loginfo("Blue cone detection with LiDAR and camera started.")
    rospy.spin()


if __name__ == "__main__":
    main()
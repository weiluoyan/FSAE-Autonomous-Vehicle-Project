#!/usr/bin/env python3
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
import open3d as o3d

# ROS publisher (init will be moved below in main)
cone_marker_pub = None

# ---- Parameters ----
CONE_HEIGHT_RANGE = (0.05, 0.18)
CONE_BOTTOM_WIDTH_RANGE = (0.08, 0.45)
CONE_TOP_WIDTH_RANGE = (0.020, 0.10)
CONE_HEIGHT_TO_BASE_RATIO = (0.3, 0.8)

DBSCAN_EPS = 0.12
DBSCAN_MIN_SAMPLES = 2

ROI_X_MIN, ROI_X_MAX = -1.0, 5.0
ROI_Y_MIN, ROI_Y_MAX = -3.0, 3.0
ROI_Z_MIN = -0.05

LIDAR_FRAME = "Pandar40P"


def remove_ground(lidar_points):
    rospy.loginfo("Starting ground removal...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_points)
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.007, ransac_n=3, num_iterations=1000)
    non_ground = pcd.select_by_index(inliers, invert=True)
    rospy.loginfo(f"Ground removal complete. Kept {len(non_ground.points)} non-ground points.")
    return np.asarray(non_ground.points)


def is_cone_shape(cluster_points):
    min_z = np.min(cluster_points[:, 2])
    max_z = np.max(cluster_points[:, 2])
    height = max_z - min_z
    base_width_x = np.ptp(cluster_points[:, 0])
    base_width_y = np.ptp(cluster_points[:, 1])
    base_width = max(base_width_x, base_width_y)
    top_width_x = np.percentile(cluster_points[:, 0], 90) - np.percentile(cluster_points[:, 0], 10)
    top_width_y = np.percentile(cluster_points[:, 1], 90) - np.percentile(cluster_points[:, 1], 10)
    top_width = max(top_width_x, top_width_y)

    if not (CONE_HEIGHT_RANGE[0] <= height <= CONE_HEIGHT_RANGE[1]):
        return False
    if not (CONE_BOTTOM_WIDTH_RANGE[0] <= base_width <= CONE_BOTTOM_WIDTH_RANGE[1]):
        return False
    if not (CONE_TOP_WIDTH_RANGE[0] <= top_width <= CONE_TOP_WIDTH_RANGE[1]):
        return False
    if not (CONE_HEIGHT_TO_BASE_RATIO[0] <= height / base_width <= CONE_HEIGHT_TO_BASE_RATIO[1]):
        return False

    return True


def publish_cones(cone_positions):
    marker_array = MarkerArray()
    for idx, position in enumerate(cone_positions):
        marker = Marker()
        marker.header.frame_id = LIDAR_FRAME
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


def process_lidar_data(point_cloud):
    rospy.loginfo("Processing incoming LiDAR data...")
    points = np.array([p[:3] for p in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)])
    rospy.loginfo(f"Total points received: {len(points)}")

    non_ground_points = remove_ground(points)
    roi_mask = (
        (non_ground_points[:, 0] > ROI_X_MIN) & (non_ground_points[:, 0] < ROI_X_MAX) &
        (non_ground_points[:, 1] > ROI_Y_MIN) & (non_ground_points[:, 1] < ROI_Y_MAX) &
        (non_ground_points[:, 2] > ROI_Z_MIN)
    )
    roi_points = non_ground_points[roi_mask]
    rospy.loginfo(f"{len(roi_points)} points after ROI filtering")

    if roi_points.shape[0] == 0:
        return

    clustering = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(roi_points)
    labels = clustering.labels_

    detected_cones = []
    for label in set(labels):
        if label == -1:
            continue
        cluster_pts = roi_points[labels == label]
        if is_cone_shape(cluster_pts):
            centroid = np.mean(cluster_pts, axis=0)
            detected_cones.append(centroid)

    rospy.loginfo(f"Detected {len(detected_cones)} cones.")
    publish_cones(detected_cones)


if __name__ == '__main__':
    rospy.init_node('cone_detector')
    cone_marker_pub = rospy.Publisher('/detected_cones', MarkerArray, queue_size=10)
    rospy.Subscriber('/velodyne_points', PointCloud2, process_lidar_data)
    rospy.loginfo("Cone detection node started.")
    rospy.spin()


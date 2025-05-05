#!/usr/bin/env python3
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
import open3d as o3d

# ---------------- CONFIG PARAMETERS ----------------
CONE_HEIGHT_RANGE = (0.02, 0.43)
CONE_BOTTOM_WIDTH_RANGE = (0.10, 0.28)
CONE_TOP_WIDTH_RANGE = (0.02, 0.12)
CONE_HEIGHT_TO_BASE_RATIO = (0.1, 2.5)

DBSCAN_EPS = 0.22               # Adjusted
DBSCAN_MIN_SAMPLES = 2

ROI_X_MIN, ROI_X_MAX = 0.8, 3.8
ROI_Y_MIN, ROI_Y_MAX = -2.0, 2.0    # Wider Y-range for left/right cones
ROI_Z_MIN = -0.3                   # Lower Z-min to catch cone bases

LIDAR_FRAME = "Pandar40P"

# ---------------- ROS Publishers ----------------
cone_marker_pub = None
cluster_debug_pub = None


def remove_ground(lidar_points):
    rospy.loginfo(" Removing ground...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_points)
    _, inliers = pcd.segment_plane(distance_threshold=0.007, ransac_n=3, num_iterations=1000)
    non_ground = pcd.select_by_index(inliers, invert=True)
    rospy.loginfo(f" Ground removed: {len(non_ground.points)} non-ground points kept.")
    return np.asarray(non_ground.points)


def is_cone_shape(cluster_points):
    if cluster_points.shape[0] < 30:
        rospy.logwarn(" Cluster too small.")
        return False

    z_values = cluster_points[:, 2]
    min_z = np.percentile(z_values, 5)
    max_z = np.percentile(z_values, 95)
    height = max_z - min_z
    base_width_x = np.ptp(cluster_points[:, 0])
    base_width_y = np.ptp(cluster_points[:, 1])
    base_width = max(base_width_x, base_width_y)
    top_width_x = np.percentile(cluster_points[:, 0], 90) - np.percentile(cluster_points[:, 0], 10)
    top_width_y = np.percentile(cluster_points[:, 1], 90) - np.percentile(cluster_points[:, 1], 10)
    top_width = max(top_width_x, top_width_y)

    rospy.loginfo(f"[ DEBUG] Height: {height:.3f}, Base: {base_width:.3f}, Top: {top_width:.3f}, Ratio: {height/base_width:.2f}")

    if not (CONE_HEIGHT_RANGE[0] <= height <= CONE_HEIGHT_RANGE[1]):
        rospy.logwarn(f" Rejected (height): {height:.3f}")
        return False
    if not (CONE_BOTTOM_WIDTH_RANGE[0] <= base_width <= CONE_BOTTOM_WIDTH_RANGE[1]):
        rospy.logwarn(f" Rejected (base width): {base_width:.3f}")
        return False
    if not (CONE_TOP_WIDTH_RANGE[0] <= top_width <= CONE_TOP_WIDTH_RANGE[1]):
        rospy.logwarn(f" Rejected (top width): {top_width:.3f}")
        return False
    if base_width == 0 or not (CONE_HEIGHT_TO_BASE_RATIO[0] <= height / base_width <= CONE_HEIGHT_TO_BASE_RATIO[1]):
        rospy.logwarn(f" Rejected (height/base ratio): {height/base_width:.2f}")
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


def publish_cluster_debug(roi_points, labels):
    marker_array = MarkerArray()
    for label in set(labels):
        if label == -1:
            continue
        cluster_pts = roi_points[labels == label]
        centroid = np.mean(cluster_pts, axis=0)

        marker = Marker()
        marker.header.frame_id = LIDAR_FRAME
        marker.header.stamp = rospy.Time.now()
        marker.ns = "debug_clusters"
        marker.id = int(label)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = centroid[2]
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)

    cluster_debug_pub.publish(marker_array)


def process_lidar_data(point_cloud):
    rospy.loginfo(" Processing LiDAR frame...")
    points = np.array([p[:3] for p in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)])
    rospy.loginfo(f" Total raw points: {len(points)}")

    non_ground_points = remove_ground(points)

    rospy.loginfo(f"📏 ROI X: {ROI_X_MIN}–{ROI_X_MAX}, Y: {ROI_Y_MIN}–{ROI_Y_MAX}, Z > {ROI_Z_MIN}")

    roi_mask = (
        (non_ground_points[:, 0] > ROI_X_MIN) & (non_ground_points[:, 0] < ROI_X_MAX) &
        (non_ground_points[:, 1] > ROI_Y_MIN) & (non_ground_points[:, 1] < ROI_Y_MAX) &
        (non_ground_points[:, 2] > ROI_Z_MIN)
    )
    roi_points = non_ground_points[roi_mask]
    rospy.loginfo(f"🔎 ROI points retained: {len(roi_points)}")

    if roi_points.shape[0] == 0:
        rospy.logwarn(" No points in ROI. Skipping frame.")
        return

    clustering = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(roi_points)
    labels = clustering.labels_

    publish_cluster_debug(roi_points, labels)

    detected_cones = []
    for label in set(labels):
        if label == -1:
            continue
        cluster_pts = roi_points[labels == label]
        if is_cone_shape(cluster_pts):
            centroid = np.mean(cluster_pts, axis=0)
            rospy.loginfo(f" Valid cone at: {centroid}")
            detected_cones.append(centroid)

    rospy.loginfo(f" Total cones detected: {len(detected_cones)}")
    publish_cones(detected_cones)


if __name__ == '__main__':
    rospy.init_node('cone_detector_debug')
    cone_marker_pub = rospy.Publisher('/detected_cones', MarkerArray, queue_size=10)
    cluster_debug_pub = rospy.Publisher('/debug_clusters', MarkerArray, queue_size=10)
    rospy.Subscriber('/velodyne_points', PointCloud2, process_lidar_data)
    rospy.loginfo(" Cone detection node (debug mode) started.")
    rospy.spin()

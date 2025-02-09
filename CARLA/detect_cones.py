import carla
import numpy as np
from sklearn.cluster import DBSCAN

def process_lidar_data(lidar_data, world):
	points = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape(-1, 4)
	xyz = points[:, :3]

	# Expanded ROI
	cone_roi = (xyz[:, 0] > -5) & (xyz[:, 0] < 60) & (xyz[:, 1] > -20) & (xyz[:, 1] < 20)
	cone_points = xyz[cone_roi]

	print(f"Points in ROI: {cone_points.shape[0]}")

	# Visualize ROI points
	for point in cone_points:
        world.debug.draw_point(
        	carla.Location(x=float(point[0]), y=float(point[1]), z=float(point[2])),
        	size=0.1,
        	color=carla.Color(255, 0, 0),  # Red
        	life_time=1.0
    	)

	if cone_points.shape[0] == 0:
    	print("No points in ROI. Skipping clustering.")
    	return

	# DBSCAN with adjusted parameters
	clustering = DBSCAN(eps=1.2, min_samples=3).fit(cone_points)
	labels = clustering.labels_

	print(f"Cluster labels: {set(labels)}")

	# Count valid clusters
	valid_cones = [label for label in set(labels) if label != -1 and np.sum(labels == label) > 5]
	num_cones = len(valid_cones)

	print(f"Detected {num_cones} cones.")

	# Visualize clusters
	colors = [
    	carla.Color(255, 0, 0),
    	carla.Color(0, 255, 0),
    	carla.Color(0, 0, 255),
    	carla.Color(255, 255, 0),
    	carla.Color(0, 255, 255)
	]

	for label in set(labels):
    	color = carla.Color(128, 128, 128) if label == -1 else colors[label % len(colors)]
    	cluster_points = cone_points[labels == label]
    	for point in cluster_points:
        	world.debug.draw_point(
            	carla.Location(x=float(point[0]), y=float(point[1]), z=float(point[2])),
            	size=0.1,
            	color=color,
            	life_time=1.0
        	)

def main():
	client = carla.Client('localhost', 2000)
	client.set_timeout(10.0)
	world = client.get_world()
	blueprint_library = world.get_blueprint_library()

	vehicle_bp = blueprint_library.filter('vehicle.*')[0]
	spawn_point = carla.Transform(carla.Location(x=230, y=195, z=0.5))
	vehicle = world.spawn_actor(vehicle_bp, spawn_point)
	spectator = world.get_spectator()
	spectator.set_transform(carla.Transform(vehicle.get_location() + carla.Location(z=20), carla.Rotation(pitch=-90)))

	lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
	lidar_bp.set_attribute('range', '50')
	lidar_bp.set_attribute('rotation_frequency', '10')
	lidar_bp.set_attribute('points_per_second', '100000')

	lidar_transform = carla.Transform(carla.Location(x=0, y=0, z=2))
	lidar_sensor = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

	lidar_sensor.listen(lambda data: process_lidar_data(data, world))

	try:
    	while True:
        	world.wait_for_tick()
	except KeyboardInterrupt:
    	lidar_sensor.stop()
    	lidar_sensor.destroy()
    	vehicle.destroy()

if __name__ == "__main__":
	main()

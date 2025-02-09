import carla
import random

def main():
	lidar_sensor = None  # Initialize to avoid UnboundLocalError
	vehicle = None   	# Initialize to avoid UnboundLocalError

	try:
    	# Step 1: Connect to the CARLA server
        client = carla.Client('localhost', 2000)
    	client.set_timeout(10.0)
    	world = client.get_world()

    	# Step 2: Get blueprint library
    	blueprint_library = world.get_blueprint_library()

    	# Step 3: Check for existing vehicles
    	vehicles = world.get_actors().filter('vehicle.*')
    	if len(vehicles) == 0:
        	print("No vehicles found. Spawning a new vehicle...")

        	# Spawn a random vehicle at a specific spawn point
        	vehicle_bp = blueprint_library.filter('vehicle.*')[0]  # Get the first vehicle blueprint
        	spawn_point = carla.Transform(carla.Location(x=230, y=195, z=0.5))  # Specific coordinates
        	vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        	print(f"Spawned a vehicle: {vehicle.type_id} at {spawn_point.location}")
    	else:
        	vehicle = vehicles[0]
        	print(f"Using existing vehicle: {vehicle.type_id}")

    	# Step 4: Move spectator camera to vehicle
    	spectator = world.get_spectator()
    	spectator_transform = carla.Transform(vehicle.get_location() + carla.Location(z=20), carla.Rotation(pitch=-90))
    	spectator.set_transform(spectator_transform)

    	# Step 5: Configure the LiDAR sensor
    	lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    	lidar_bp.set_attribute('range', '50')  # Detection range in meters
    	lidar_bp.set_attribute('rotation_frequency', '10')  # Rotations per second
    	lidar_bp.set_attribute('points_per_second', '100000')  # Number of points

    	# Step 6: Attach LiDAR to the vehicle
    	lidar_transform = carla.Transform(carla.Location(x=0, y=0, z=2))  # LiDAR placement
    	lidar_sensor = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
    	print("LiDAR sensor attached to the vehicle.")

    	# Step 7: Listen for LiDAR data
    	def lidar_callback(data):
        	print("LiDAR data received")

    	lidar_sensor.listen(lidar_callback)
    	print("LiDAR sensor is active. Press Ctrl+C to stop.")

    	# Keep the script running
    	while True:
        	world.wait_for_tick()

	except KeyboardInterrupt:
    	print("\nScript stopped by user.")

	finally:
    	# Cleanup
    	if lidar_sensor:
        	lidar_sensor.stop()
        	lidar_sensor.destroy()
        	print("LiDAR sensor destroyed.")
    	if vehicle:
        	vehicle.destroy()
        	print("Vehicle destroyed.")

if __name__ == "__main__":
	main()

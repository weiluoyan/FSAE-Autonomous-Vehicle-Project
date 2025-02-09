import carla

def main():
	try:
    	# Step 1: Connect to the CARLA server
    	client = carla.Client('localhost', 2000)
    	client.set_timeout(10.0)

    	# Step 2: Get the world
    	world = client.get_world()

    	# Step 3: Load the blueprint library
    	blueprint_library = world.get_blueprint_library()

    	# Step 4: Use the correct cone blueprint (trafficcone01 or trafficcone02)
    	cone_blueprint = blueprint_library.find('static.prop.trafficcone01')

    	# Step 5: Define spawn points for cones (adjust to map's visible area)
    	spawn_points = [
        	carla.Transform(carla.Location(x=0 + i * 2, y=0, z=0.5)) for i in range(10)
    	]

    	# Step 6: Move the spectator camera to view the cones
    	spectator = world.get_spectator()
    	spectator.set_transform(carla.Transform(carla.Location(x=10, y=0, z=10), carla.Rotation(pitch=-45)))

    	# Step 7: Spawn cones at defined locations and debug the process
    	actors = []
    	for i, spawn_point in enumerate(spawn_points):
        	actor = world.try_spawn_actor(cone_blueprint, spawn_point)
        	if actor:
            	print(f"Cone {i + 1} spawned at {spawn_point.location}")
            	actors.append(actor)
        	else:
            	print(f"Failed to spawn cone {i + 1} at {spawn_point.location}")

    	# Step 8: Draw debug points where the cones are spawned
    	for spawn_point in spawn_points:
        	world.debug.draw_point(spawn_point.location, size=0.5, color=carla.Color(255, 0, 0), life_time=10.0)

    	print(f"Spawned {len(actors)} cones in the simulation.")
    	print("You can check the CARLA simulator window to see the cones.")

    	# Wait for the user to finish
    	input("Press Enter to remove the cones and exit...")

	finally:
    	# Cleanup actors on exit
    	for actor in actors:
        	actor.destroy()
    	print("All cones have been removed.")

# Run the main function
if __name__ == "__main__":
	main()


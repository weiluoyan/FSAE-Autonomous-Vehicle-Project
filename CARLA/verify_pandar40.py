import carla
import rospy
from sensor_msgs.msg import PointCloud2
import json
from websocket import create_connection

# ROSbridge WebSocket configuration
CARLA_WS_URL = "ws://localhost:9090"  # Adjust WebSocket URL and port if needed

# ROS Callback to handle LiDAR data
def lidar_callback(data):
    try:
        print("Received LiDAR data from Pandar40P!")

        # Serialize PointCloud2 data (or adapt serialization for CARLA)
        lidar_data_json = {
            "header": {
                "seq": data.header.seq,
                "stamp": str(data.header.stamp),
                "frame_id": data.header.frame_id
            },
            "height": data.height,
            "width": data.width,
            "fields": [{"name": field.name, "offset": field.offset, "datatype": field.datatype, "count": field.count} for field in data.fields],
            "is_bigendian": data.is_bigendian,
            "point_step": data.point_step,
            "row_step": data.row_step,
            "data": list(data.data),  # This may require compression for large datasets
            "is_dense": data.is_dense
        }

        # Send data to CARLA via WebSocket
        carla_ws.send(json.dumps(lidar_data_json))
        print("LiDAR data sent to CARLA successfully!")

    except Exception as e:
        print(f"Error processing LiDAR data: {e}")

def verify_pandar40():
    global carla_ws
    try:
        # Connect to CARLA
        client = carla.Client("localhost", 2000)  # Adjust host and port if needed
        client.set_timeout(10.0)
        world = client.get_world()
        print("Connected to CARLA.")

        # Add Pandar40P LiDAR Sensor (if not already present)
        blueprint_library = world.get_blueprint_library()
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '40')  # Simulate Pandar40P-specific parameters
        lidar_bp.set_attribute('points_per_second', '2000000')  # Adjust as needed
        lidar_bp.set_attribute('range', '100.0')  # Adjust range for Pandar40P

        # Position and spawn the LiDAR sensor
        spawn_point = carla.Transform(carla.Location(x=0, y=0, z=2))  # Adjust as needed
        lidar_sensor = world.spawn_actor(lidar_bp, spawn_point)
        print("Pandar40P LiDAR sensor added to the simulation!")

        # List all actors (sensors, vehicles, etc.)
        actors = world.get_actors()
        for actor in actors:
            print(f"Actor ID: {actor.id}, Type: {actor.type_id}")

        sensors = [actor for actor in actors if 'sensor' in actor.type_id.lower()]
        print("Available sensors in CARLA:")
        for sensor in sensors:
            print(f" - {sensor.type_id}")

        # Verify Pandar40P
        has_pandar40 = any("ray_cast" in sensor.type_id.lower() for sensor in sensors)
        if has_pandar40:
            print("Pandar40P LiDAR is available in CARLA.")
        else:
            print("Pandar40P LiDAR is NOT available in CARLA. Please check configuration.")
            return

        # Initialize ROS Node
        rospy.init_node('pandar40_verification', anonymous=True)

        # Subscribe to the Pandar40P LiDAR topic
        lidar_topic = "/hesai_lidar"  # Update to match your CARLA-ROS bridge configuration
        rospy.Subscriber(lidar_topic, PointCloud2, lidar_callback)
        print(f"Subscribed to LiDAR topic: {lidar_topic}")

        # Connect to ROSbridge WebSocket
        print("Connecting to ROSbridge WebSocket...")
        carla_ws = create_connection(CARLA_WS_URL)
        print(f"Connected to ROSbridge WebSocket at {CARLA_WS_URL}")

        # Keep the script running until data is received
        print("Waiting for LiDAR data...")
        rospy.spin()

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if carla_ws:
            carla_ws.close()
            print("Closed connection to ROSbridge WebSocket.")

if __name__ == "__main__":
    verify_pandar40()
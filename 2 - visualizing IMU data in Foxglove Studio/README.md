# Visualizing /mavros/imu/data in Foxglove Studio

### 1. Start rosbridge_server
If not already running:
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
It should say: `Rosbridge WebSocket server started on port 9090`

### 2. Open Foxglove Studio
- Go to: https://studio.foxglove.dev/
- Click `Connect to data source`
- Choose "ROS 2 WebSocket"

If it's a local machine:
```
ws://localhost:9090
```
Or use the IP of the ROS host if accessing remotely:
```
ws://<your-ip>:9090
```
### 3. Add Visualization Panels
Once connected:
- Click “+ Add panel”
- Choose "3D"
- In the 3D panel settings:
  - Add topic: /mavros/imu/data
  - Topic type: sensor_msgs/Imu
  - Visualize as: IMU Arrows or Orientation

# 4. Customize the View
- Use mouse to rotate/zoom
- Tweak the style settings (scale, color)
- Optionally, add **Raw Messages** panel to inspect values

# Visualizing /mavros/imu/data in Foxglove Studio

## Foxglove
Foxglove is a powerful visualization and debugging tool for robotics and autonomous systems, often used with ROS (Robot Operating System). It helps developers inspect, analyze, and interact with real-time data from their robots.It is a platform to record, upload, organize, and visualize multimodal log data such as time series, text logs, video, 3D, maps, and more. It is most often used in hardware, robotics, and physical AI.

!(0)[https://github.com/syedmohiuddinzia/ros2_orangeCube/blob/main/2%20-%20visualizing%20IMU%20data%20in%20Foxglove%20Studio/pic/0.png]

### What Foxglove Does
- Foxglove provides:
  - 3D visualization of robot data (IMU, LiDAR, transforms, etc.)
  - Topic inspection (view ROS topics and messages live)
  - Time-series plots for sensor data
  - Map and camera feeds
  - Panel-based layout (customizable dashboard)
  - Playback of recorded data (e.g., ROS bag files)
  - Remote robot debugging
- Compatible With
  - ROS 1 and ROS 2
  - MCAP files (newer alternative to ROS bags)
  - WebSockets, ROS bridges, and Data logs
  - Sensors like IMUs, GPS, LiDAR, cameras, etc.
- Two Versions
  - Foxglove Studio (Desktop App)
    - Full-featured GUI with plugins
      - Runs on Windows, macOS, Linux
    - Foxglove Web
      - Runs in a browser
      - Useful for remote access to robot data
- Example Use Cases
  - Debugging a robot’s navigation stack
  - Visualizing a drone’s flight path and orientation
  - Monitoring a self-driving car’s perception system
  - Inspecting robot sensor logs

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

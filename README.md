# NewMIR
NewMIR is a project focused on autonomous robot navigation using ROS2, featuring waypoint
management and WebSocket communication for real-time updates.
## Features
- Autonomous navigation with predefined waypoints
- Real-time communication using WebSockets
- Integration with Vue.js frontend for control interface
- Speed control for virtual joystick and navigation
## Getting Started
### Prerequisites
- ROS2 (Foxy, Galactic, or Humble)
- TurtleBot3
- Gazebo
- Node.js
- Vue.js
### Installation
1. **Clone the repository:**
 ```sh
 git clone git@github.com:deenqtt/NewMIR.git
 cd NewMIR
 ```
2. **Install ROS2 dependencies:**
 ```sh
 rosdep install --from-paths src --ignore-src -r -y
 colcon build
 ```
3. **Install Node.js dependencies:**
 ```sh
 cd web_interface
 npm install
 ```
### Usage
1. **Launch ROS2:**
 ```sh
 source /opt/ros/<your_ros2_distro>/setup.bash
 source install/setup.bash
 ros2 launch newmir_bringup newmir.launch.py
 ```
2. **Start the WebSocket server:**
 ```sh
 node server.js
 ```
3. **Run the Vue.js frontend:**
 ```sh
 cd web_interface
 npm run serve
 ```
### Configuration
- **ROS2 Parameters:**
 Adjust parameters in `params.yaml` for different components like AMCL, Nav2, etc.
- **Web Interface:**
 Configure settings in `web_interface/src/config.js`.
### Troubleshooting
- **Common Issues:**
 - Ensure all dependencies are installed correctly.
 - Check network configurations for WebSocket communication.
### Contributing
1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Commit your changes (`git commit -m 'Add some feature'`).
4. Push to the branch (`git push origin feature-branch`).
5. Open a pull request.
### License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

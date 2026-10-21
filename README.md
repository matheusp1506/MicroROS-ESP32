# micro-ROS ESP32 Project

This project runs a **micro-ROS** client on the ESP32 using ESP-IDF. It connects to your local Wi-Fi network and streams simulated robot sensor data (`nav_msgs/msg/Odometry` and `sensor_msgs/msg/Imu`) to a micro-ROS Agent via UDP.

---

## Features

- **Wi-Fi Connectivity**: Connects to your local Wi-Fi station network.
- **micro-ROS over UDP**: Communicates with a micro-ROS Agent running on your host machine / ROS 2 network.
- **Sensors Streamed**:
  - `/odom` (`nav_msgs/msg/Odometry`) at 20 Hz
  - `/imu` (`sensor_msgs/msg/Imu`) at 20 Hz
- **Secure Configuration via Kconfig**: Wi-Fi credentials and Agent IP/port are kept out of source code and Git.

---

## Project Structure

```text
├── CMakeLists.txt
├── colcon.meta
├── components/
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml
│   ├── Kconfig.projbuild      # Menuconfig definitions for Wi-Fi & Agent
│   └── main.c                 # Application entry point & micro-ROS logic
├── sdkconfig                  # Local config (ignored by Git)
└── README.md
```

---

## Configuration

Project configuration (Wi-Fi credentials and Agent endpoints) is managed through **ESP-IDF Kconfig**. Since `sdkconfig` is gitignored, your sensitive credentials will never be committed to Git.

1. Open the configuration menu:
   ```bash
   idf.py menuconfig
   ```
   *(or use the ESP-IDF Extension gear icon in VS Code)*

2. Navigate to:
   ```text
   Micro-ROS & Wi-Fi Configuration --->
   ```

3. Set your parameters:
   - **WiFi SSID**: Your Wi-Fi network name.
   - **WiFi Password**: Your Wi-Fi password.
   - **Micro-ROS Agent IP**: IP address of the machine running the micro-ROS agent (e.g., `192.168.1.100`).
   - **Micro-ROS Agent Port**: UDP port of the agent (default: `8888`).

4. Press **`S`** to save and **`Q`** to exit.

---

## Running the micro-ROS Agent

Before or right after powering the ESP32, start the micro-ROS agent on your PC/host machine:

### Using Docker (Recommended):
```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
```
*(Replace `humble` with your ROS 2 distribution if different, e.g., `iron`, `jazzy`)*

### Using ROS 2 directly:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

---

## Build, Flash, and Monitor

1. Set the target chip (if not already set):
   ```bash
   idf.py set-target esp32
   ```

2. Build the project:
   ```bash
   idf.py build
   ```

3. Flash and open the serial monitor (adjust `/dev/ttyUSB0` to your port):
   ```bash
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

To exit the serial monitor, press `Ctrl + ]`.

---

## Verifying Topics in ROS 2

Once connected, verify the topics on your ROS 2 workstation:

```bash
# List active topics
ros2 topic list

# Echo Odometry
ros2 topic echo /odom

# Echo IMU
ros2 topic echo /imu
```

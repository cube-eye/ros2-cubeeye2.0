# CUBE EYE camera ROS2 driver (S100D / S110D / S111D / I200D device)
This ROS2 driver package supports the following CUBE EYE devices:
- S100D / S110D / S111D / I200D

For detailed product specifications, please visit [Cube Eye](http://www.cube-eye.co.kr).

## Installation Instructions

These instructions are written for ROS2 Humble on Ubuntu 22.04. They were previously tested on ROS2 Eloquent with Ubuntu 18.04.

### Step 1: Install ROS2 Distribution

- Install ROS2 Humble distribution from [here](https://docs.ros.org/en/rolling/Releases.html).

### Step 2: Install the Driver

- Create a workspace:
  ```bash
  $ source /opt/ros/humble/setup.bash
  $ mkdir -p ~/dev_ws/src
  $ cd ~/dev_ws/src
  ```

- Copy the driver source into the workspace (`dev_ws/src`).

- Update CubeEye2.0 SDK

  Remove the old SDK:
  ```bash
  $ rm -rf src/ros2-cubeeye2.0/cubeeye2.0
  ```
  Download CubeEye2.0 SDK from [Cube Eye](http://www.cube-eye.co.kr) and install it into `cubeeye2.0` directory:
  ```bash
  $ tar xvzf xxx-xxx-xxx.tar.gz
  $ cp -rf xxx-xxx-xxx/release src/ros2-cubeeye2.0/cubeeye2.0
  ```

  (SDK version information can be found in `xxx-xxx-xxx/release/doc/html/index.html`)

- Build the workspace:
  ```bash
  $ cd ~/dev_ws
  $ colcon build --symlink-install
  ```

## Usage Instructions

Connect the camera power and execute the following commands:

```bash
$ source install/setup.bash
```

Run the camera node:
```bash
$ ros2 run cubeeye_camera cubeeye_camera_node
```

Or launch using a launch file:
```bash
$ ros2 launch cubeeye_camera cubeeye_camera_launch.py
```

### Services

#### Scan CUBE EYE Camera
```bash
$ ros2 service call /cubeeye_camera_node/scan cubeeye_camera/srv/Scan
```

#### Connect Camera Source (Camera Index)
Connect the first camera (index: 0) in scanned sources:
```bash
$ ros2 service call /cubeeye_camera_node/connect cubeeye_camera/srv/Connect "{index: 0}"
```

#### Run Camera (Frame Type)
Run Amplitude and Depth (type: 6):
```bash
$ ros2 service call /cubeeye_camera_node/run cubeeye_camera/srv/Run "{type: 6}"
```
Or run PointCloud (type: 32):
```bash
$ ros2 service call /cubeeye_camera_node/run cubeeye_camera/srv/Run "{type: 32}"
```

#### Stop Camera
```bash
$ ros2 service call /cubeeye_camera_node/stop cubeeye_camera/srv/Stop
```

#### Disconnect Camera
```bash
$ ros2 service call /cubeeye_camera_node/disconnect cubeeye_camera/srv/Disconnect
```

### Topics

Once a camera is connected and started with a frame type, the following topics are published:

- `/cubeeye_camera_node/depth`: Depth Image
- `/cubeeye_camera_node/amplitude`: Amplitude Image
- `/cubeeye_camera_node/rgb`: RGB Image
- `/cubeeye_camera_node/points`: PointCloud Image

### Operating Test

Launch `rqt` for visualizing amplitude and depth images:
```bash
$ rqt
```
/cubeeye_camera_node/amplitude, /cubeeye_camera_node/depth
<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/104545764-ca58ce00-55f8-11eb-86a9-3bc091a1aeb9.png"/></p>

Visualize PointCloud using RViz2:
```bash
$ ros2 run rviz2 rviz2
```
Set Fixed Frame to `pcl` and view PointCloud2 at `/cubeeye_camera_node/points`.

<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/104545815-de043480-55f8-11eb-8293-baa2edba664f.png"/></p>

### Using Dynamic Reconfigure Params

Launch `rqt_reconfigure` to configure parameters dynamically:
```bash
$ ros2 run rqt_reconfigure rqt_reconfigure
```
![dynamic_reconfigure](https://user-images.githubusercontent.com/90016619/131959624-b3ff03e2-2eb7-4cc6-bc5d-df39dc74ed89.png)

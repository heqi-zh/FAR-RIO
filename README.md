# FAR-RIO

**A Fast and Robust Radar-Inertial Odometry with Isotropic Uncertainty Model and Dual-Observation Update Pipeline**

[Paper](https://ieeexplore.ieee.org/abstract/document/11474877) · [Installation](#installation) · [Configuration](#configuration) · [Run](#run) · [License](#license)

FAR-RIO is the official implementation of the paper linked above. It estimates motion from 4D radar point clouds, Doppler measurements, and IMU data. The method combines dynamic point filtering, an isotropic uncertainty model for registration, and a dual-observation update pipeline with keyframe selection.

The ROS package and C++ namespace are both named `far_rio`. This repository includes radar input adapters for **Snail** and **NTU**.

## Installation

### Dependencies

The commands below use an Ubuntu 20.04 / ROS Noetic environment. Install ROS first, then install the build dependencies:

```bash
sudo apt update
sudo apt install build-essential cmake libgoogle-glog-dev libeigen3-dev \
    libpcl-dev libyaml-cpp-dev libtbb-dev \
    ros-noetic-pcl-ros ros-noetic-eigen-conversions ros-noetic-rosbag \
    ros-noetic-tf ros-noetic-rviz
```

A compiler with C++17 support is required; use GCC 9 or newer. PCL must be version 1.8 or newer.

### Build

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/heqi-zh/FAR-RIO.git far_rio
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

The default voxel representation can be replaced with PHC voxels at build time:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DWITH_IVOX_NODE_TYPE_PHC=ON
```

For an environment that needs the bundled TBB 2018 library, extract `thirdparty/tbb2018_20170726oss_lin.tgz` and pass its extracted directory to CMake using `-DCUSTOM_TBB_DIR=/absolute/path/to/tbb2018_20170726oss`.

## Configuration

Choose the YAML file matching your radar message format:

| Dataset / adapter | Configuration | Launch file | Radar message |
| --- | --- | --- | --- |
| Snail | [Snail_radar.yaml](config/Snail_radar.yaml) | [Snail.launch](launch/Snail.launch) | `sensor_msgs/PointCloud2` |
| NTU | [NTU_radar.yaml](config/NTU_radar.yaml) | [NTU.launch](launch/NTU.launch) | `sensor_msgs/PointCloud` |

Both adapters use `sensor_msgs/Imu` for IMU input. Set the radar and IMU topics, sensor extrinsics, and noise parameters in the selected YAML file before running.

### Topics and calibration

For example, the Snail configuration contains:

```yaml
common:
    radar_topic: "/ars548"
    imu_topic: "/x36d/imu_raw"

preprocess:
    radar_type: 4                # 4: Snail; 5: NTU

mapping:
    extrinsic_est_en: true
    extrinsic_T: [0, 0, 0]
    extrinsic_R: [0.998861908053375, 0.0445688670331839, -0.0169854270287433,
                  -0.0445188967507348, 0.999003062288094, 0.00330898339477966,
                   0.0171159712569224, -0.0025490449952668, 0.999850261737998]
```

`extrinsic_R` is a row-major rotation matrix and `extrinsic_T` is the translation from the radar frame to the IMU frame:

```text
p_imu = extrinsic_R * p_radar + extrinsic_T
```

Replace the supplied calibration with the calibration for your sensor setup. Radar and IMU messages must have synchronized timestamps.

### Data and output paths

Configure offline input and output paths in the same YAML file:

```yaml
input:
    bag_file: ""                 # Set the path to your ROS bag for offline processing.

output:
    directory: "../output"
```

For example, set `bag_file` to `"/data/snail/sequence.bag"` or `"../data/sequence.bag"`. YAML paths may be absolute or relative to the directory containing that YAML file. With the supplied configurations, `../output` points to the repository's `output/` directory. Use an absolute output directory if you prefer to save results elsewhere. Output directories are created automatically.

`input.bag_file` is left empty in the repository because it is specific to your machine. It is required only by the offline launch. There are no required command-line path flags.

### Radar message formats

The input adapters are implemented in [pointcloud_preprocess.cc](src/pointcloud_preprocess.cc):

- **Snail:** reads Cartesian coordinates, intensity, and Doppler from a `PointCloud2`. The adapter accepts the `Doppler` or `doppler` field and reads the available `range_std`, `azimuth_std`, `elevation_std`, and `doppler_std` fields. The full registered point layout is defined in [common_lib.h](include/common_lib.h).
- **NTU:** reads Cartesian coordinates from `points` and uses `channels[0]` for Doppler, `channels[1]` for range, `channels[3]` for azimuth, and `channels[4]` for elevation. The angle channels are in degrees. Keep this channel ordering when preparing a bag.

The adapters include dataset-specific Doppler and coordinate conventions. For another radar or message layout, adapt the corresponding parser and calibration. `config/HKUST.yaml` is retained as an inactive reference and has no supported input adapter.

## Run

Source `~/catkin_ws/devel/setup.bash` in each terminal before launching.

### Online: live sensors or bag replay

Start the radar and IMU publishers, then launch the matching configuration:

```bash
roslaunch far_rio Snail.launch
```

For NTU:

```bash
roslaunch far_rio NTU.launch
```

RViz opens by default. To replay a bag automatically, edit `bag_file` in the selected launch file:

```xml
<arg name="bag_file" default="/data/snail/sequence.bag" />
```

Use an **absolute path** for this launch argument. The launch file starts `rosbag play --clock` and enables simulated ROS time when the path is set. Leave it empty for live sensors. The YAML's `input.bag_file` is used by the offline entry point only.

The launch file also selects the YAML file and controls RViz:

```xml
<arg name="config_file" default="$(find far_rio)/config/Snail_radar.yaml" />
<arg name="rviz" default="true" />
```

Set `rviz` to `false` when running without a display. The online launches retain a `filter_size_surf` override of `0.2`; adjust that launch parameter if you want a different online downsampling size.

### Offline: process a bag directly

1. Set `input.bag_file` in the selected YAML file.
2. In [offline.launch](launch/offline.launch), select that YAML file using `config_file`. The default is `Snail_radar.yaml`; change it to `NTU_radar.yaml` for NTU data.
3. Run:

```bash
roslaunch far_rio offline.launch
```

The offline entry point reads the configured radar and IMU topics directly from the bag. It does not require a separate bag player or publish visualization topics.

## Outputs

Results are written under `output.directory` when processing finishes. Stop the online node with **Ctrl+C** to finish saving its outputs.

| File | Contents |
| --- | --- |
| `trajectory.txt` | Trajectory: `timestamp tx ty tz qx qy qz qw`; poses are collected when `path_save_en` or path publishing is enabled. |
| `time.csv` | Timing records for the processing stages. |
| `Log/imu_.txt` | IMU debug log. |
| `PCD/scans.pcd` | Accumulated map when `pcd_save.pcd_save_en` is enabled. |
| `PCD/scans_<index>.pcd` | Map chunks when `pcd_save.interval` is positive. |

`pcd_save.interval: -1` accumulates the map until shutdown. A positive interval writes a chunk after that many frames. Online ROS outputs include `/Odometry`, `/path`, `/cloud_registered`, `/cloud_registered_body`, and `/cloud_registered_effect_world`; their publication is controlled by the YAML `publish` settings.

## Citation

If you use FAR-RIO in your research, please cite:

**FAR-RIO: A Fast and Robust Radar-Inertial Odometry with Isotropic Uncertainty Model and Dual-Observation Update Pipeline.** [IEEE Xplore](https://ieeexplore.ieee.org/abstract/document/11474877).

Use the citation export on the paper's IEEE Xplore page for the bibliographic entry.

## Acknowledgements

FAR-RIO builds on [Faster-LIO](https://github.com/gaoxiang12/faster-lio), [FAST-LIO](https://github.com/hku-mars/FAST_LIO), and the bundled [IKFoM toolkit](https://github.com/hku-mars/IKFoM). We thank their authors for making these projects available.

## License

FAR-RIO is distributed under **GNU GPL v2** (`GPL-2.0-only`). See [LICENSE](LICENSE) for the full text and [NOTICE](NOTICE) for project and upstream attribution. Bundled third-party code retains its original notices and applicable license terms.

## Contact

Hang Zhen — [zhenhangwhu@whu.edu.cn](mailto:zhenhangwhu@whu.edu.cn).

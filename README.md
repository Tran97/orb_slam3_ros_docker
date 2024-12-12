# ORB-SLAM3-ROS

## 1. Installation
```
git clone https://github.com/Tran97/orb_slam3_ros_docker.git
cd orb_slam3_ros_docker
docker compose build dev
```

## 2. Running container environment
```
docker compose up dev
```
```
docker exec -it orb_slam3_ros_docker-dev-1 bash
```

## 3. Run rosbag Examples

### Mono mode with [NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/)'s [`eee_01.bag`](https://researchdata.ntu.edu.sg/api/access/datafile/68133):

```
# In one terminal:
roslaunch orb_slam3_ros ntuviral_mono.launch
# In another terminal:
rosbag play eee_01.bag -s 50 # The UAV starts moving at t~50s
```
### Stereo mode with [KITTI](https://www.cvlibs.net/datasets/kitti/index.php)'s [`2011_09_26`](https://www.cvlibs.net/datasets/kitti/raw_data.php):
- First, download KITTI dataset and convert the raw data into bag file following [this instruction](https://stevenliu216.github.io/2018/08/05/working-with-kitti-ros/). You can automate the downloading process using [this script](https://github.com/Deepak3994/Kitti-Dataset).
- Run the example:
```
# In one terminal:
roslaunch orb_slam3_ros kitti_stereo.launch
# In another terminal:
rosbag play kitti_2011_09_26_drive_0002_synced.bag
```

### Mono-inertial mode with [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)'s [`MH_01_easy.bag`]( http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag):
```
# In one terminal:
roslaunch orb_slam3_ros euroc_mono_inertial.launch
# In another terminal:
rosbag play MH_01_easy.bag
```
### Stereo-inertial mode with [TUM-VI](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)'s [`dataset-corridor1_512_16.bag`](https://vision.in.tum.de/tumvi/calibrated/512_16/dataset-corridor1_512_16.bag)
```
# In one terminal:
roslaunch orb_slam3_ros tum_vi_stereo_inertial.launch
# In another terminal:
rosbag play dataset-corridor1_512_16.bag
```
### RGB-D mode with [TUM](http://vision.in.tum.de/data/datasets/rgbd-dataset/download)'s [`rgbd_dataset_freiburg1_xyz.bag`](https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.bag)
```
# In one terminal:
roslaunch orb_slam3_ros tum_rgbd.launch
# In another terminal:
rosbag play rgbd_dataset_freiburg1_xyz.bag
```
- **Note**: change `TUMX.yaml` to `TUM1.yaml`,`TUM2.yaml` or `TUM3.yaml` for freiburg1, freiburg2 and freiburg3 sequences respectively.

### RGB-D-Inertial mode with [VINS-RGBD](https://github.com/STAR-Center/VINS-RGBD)'s [`Normal.bag`](https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.bag)
- Download the bag files, for example [Normal.bag](https://star-center.shanghaitech.edu.cn/seafile/d/0ea45d1878914077ade5/).
- Decompress the bag, run `rosbag decompress Normal.bag`.
- Change the calibration params in `RealSense_D435i.yaml` if necessary.
```
# In one terminal:
roslaunch orb_slam3_ros rs_d435i_rgbd_inertial.launch.launch
# In another terminal:
rosbag play Normal.bag
```

### Live stereo-inertial mode with Realsense T265
- Modify the original `rs_t265.launch` to enable fisheye images and imu data (change `unite_imu_method` to `linear_interpolation`).
- Run `rs-enumerate-devices -c` to get the calibration parameters and modify `config/Stereo-Inertial/RealSense_T265.yaml` accordingly. A detailed explaination can be found [here](https://github.com/shanpenghui/ORB_SLAM3_Fixed#73-set-camera-intrinsic--extrinsic-parameters).
- Run:
```
# In one terminal:
roslaunch realsense2_camera rs_t265.launch
# In another terminal:
roslaunch orb_slam3_ros rs_t265_stereo_inertial.launch
```

### Save and load map 

The map file will have `.osa` extension, and is located in the `ROS_HOME` folder (`~/.ros/` by default).
#### Save map:
- **Option 1**: If `System.SaveAtlasToFile` is set in the settings file, the map file will be automatically saved when you kill the ros node.
- **Option 2**: You can also call the following ros service at the end of the session
```
rosservice call /orb_slam3/save_map [file_name]
```
#### Load map:
- Create a folder called maps in the orb_slam3_ros package.
- Move the saved map in `~/.ros` to this maps folder in the ros package.
- Specify the name of the map in the launch file as in the example `rs_rgbd.launch`.


## 4. ROS topics, params and services
### Subscribed topics
- `/camera/image_raw` for Mono(-Inertial) node
- `/camera/left/image_raw` for Stereo(-Inertial) node
- `/camera/right/image_raw` for Stereo(-Inertial) node
- `/imu` for Mono/Stereo/RGBD-Inertial node
- `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw` for RGBD node
### Published topics
- `/orb_slam3/camera_pose`, left camera pose in world frame, published at camera rate
- `/orb_slam3/body_odom`, imu-body odometry in world frame, published at camera rate
- `/orb_slam3/tracking_image`, processed image from the left camera with key points and status text
- `/orb_slam3/tracked_points`, all key points contained in the sliding window
- `/orb_slam3/all_points`, all key points in the map
- `/orb_slam3/kf_markers`, markers for all keyframes' positions
- `/tf`, with camera and imu-body poses in world frame
### Params
- `voc_file`: path to vocabulary file required by ORB-SLAM3
- `settings_file`: path to settings file required by ORB-SLAM3
- `enable_pangolin`: enable/disable ORB-SLAM3's Pangolin viewer and interface. (`true` by default)

### Services
- `rosservice call /orb_slam3/save_map [file_name]`: save the map as `[file_name].osa` in `ROS_HOME` folder.
- `rosservice call /orb_slam3/save_traj [file_name]`: save the estimated trajectory of camera and keyframes as `[file_name]_cam_traj.txt` and  `[file_name]_kf_traj.txt` in `ROS_HOME` folder.

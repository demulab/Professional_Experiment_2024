# Professional_Experiment_2024
## ピック＆プレース
### USBカメラ
```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2
```
### ARマーカ認識
```
cd ~/.ros
mkdir camera_info
default_cam.yamlをペーストする
```

```
ros2 run opencv_ros2 aruco_node_tf
```
```
ros2 run static_broadcaster broadcaster
```
```
ros2 run aruco_listener listener
```
### アーム制御
```
sudo chmod a+rw /dev/ttyUSB0

ros2 run dynamixel_sdk_examples read_write_trapezoidal_node

python3 open_manipulator_x.py
```

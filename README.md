# Professional_Experiment_2024
## Proxy設定


## インストール
```
# vision
sudo apt install ros-foxy-vision-opencv
sudo apt install ros-foxy-usb-cam
pip3 install scipy
pip3 install opencv-contrib-python==4.6.0.66 transforms3d

# tf2
sudo apt install ros-foxy-turtle-tf2-py ros-foxy-tf2-tools ros-foxy-tf-transformations
```

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

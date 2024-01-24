# Professional_Experiment_2024
## Proxy設定
```/etc/apt/```内の```apt.conf```ファイルに下記の入力する。```apt.conf```ファイルが存在しなければ作成する。
```
Acquire::http::Proxy "http://wwwproxy.kanazawa-it.ac.jp:8080";
Acquire::https::Proxy "https://wwwproxy.kanazawa-it.ac.jp:8080";
```

ubuntuの「設定」から「ネットワーク」の「ネットワークプロキシ」を手動に設定する。その時、以下の画像のように```wwwproxy.kanazawa-it.ac.jp```を追加する。


![image](https://github.com/demulab/Professional_Experiment_2024/assets/42795206/ab7a7204-06d5-4f88-aad8-8a10bb1ecccd)


## インストール
```
sudo apt update

# vision
sudo apt install ros-foxy-vision-opencv
sudo apt install ros-foxy-usb-cam
pip3 install scipy
pip3 install opencv-contrib-python==4.6.0.66 transforms3d

# tf2
sudo apt install ros-foxy-turtle-tf2-py ros-foxy-tf2-tools ros-foxy-tf-transformations
```

## ピック＆プレース
**動作動画**

https://www.youtube.com/watch?v=LFXVFafaE2M

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

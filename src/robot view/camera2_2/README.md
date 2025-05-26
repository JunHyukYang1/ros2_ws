# camera
ros2 test package for CSI camera on Jetson nano

dependency : ros2 foxy, gstreamer, opencv 4.8, cmake 3.16

Publisher node captures an image from csi camera via gstreamer and publishes a compressed image topic with jpg format using a ros2 interface sensor_msgs/msg/CompressedImage.
Subscriber node subscribes the compressed image topic and sends it to PC via gstreamer.

Open linux terminal on Jetson nano

$ cd ~/ros2_ws/src

$ git clone https://github.com/2sungryul/camera.git

Make sure you need to change the ip address in sub.cpp to that of your own PC.

$ cd ~/ros2_ws

$ colcon build --symlink-install --packages-select camera

$ source install/local_setup.bash

$ ros2 run camera pub

Open new linux terminal on Jetson nano

$ ros2 run camera sub_jetson

Open windows powershell on PC

PS> gst-launch-1.0 -v udpsrc port=8001 ! ‘application/x-rtp,encoding-name=(string)H264,payload=(int)96’ ! rtph264depay ! queue ! avdec_h264 ! videoconvert! autovideosink

![image](https://github.com/2sungryul/camera/assets/67367753/61171e79-f093-441a-ad77-ae4f7b8adc19)

Open linux terminal on WSL2

$ cd ~/ros2_ws/src

$ git clone https://github.com/2sungryul/camera.git

$ cd ~/ros2_ws

$ colcon build --symlink-install --packages-select camera

$ source install/local_setup.bash

$ ros2 run camera sub_wsl

![image](https://github.com/2sungryul/camera/assets/67367753/6f54ffa5-abcf-4848-95ed-68e45e195c46)

Gstreamer command for video streaming from csi camera on Linux 

$ gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),format=NV12,width=640,height=360' ! nvvidconv flip-method=0 ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.121 port=8001 sync=false -q

How to create Virtual Switch using Hyper-V 

![image](https://github.com/2sungryul/camera/assets/67367753/273c7a3c-af2d-40ac-a0fe-ae18f5b98443)

How to create .wslconfig for WSL2 

![image](https://github.com/2sungryul/camera/assets/67367753/1bf746e9-707b-416a-9987-02c689287da6)



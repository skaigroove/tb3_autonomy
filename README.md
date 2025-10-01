# tb3_autonomy
A complete autonomy stack for TurtleBot3 on ROS2 Humble with Raspberry Pi 4. It fuses LiDAR, RGB-D camera, and IMU data, supports YOLO-based object detection, and provides SLAM and navigation via RTAB-Map.

cd ~
git clone https://github.com/skaigroove/tb3_autonomy.git
cd ~/tb3_autonomy/robot/src
cp -r src ~/set_rasp4
cd ~/set_rasp4/
colcon build --symlink-install

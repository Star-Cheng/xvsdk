sudo apt update
sudo apt upgrade
sudo apt-get install -y libopencv-dev
sudo apt-get install -y tree g++ cmake cmake-curses-gui pkg-config autoconf libtool libudev-dev libjpeg-dev zlib1g-dev libopencv-dev rapidjson-dev libeigen3-dev libboost-thread-dev libboost-filesystem-dev libboost-system-dev libboost-program-options-dev libboost-date-time-dev libboost-chrono-dev liboctomap-dev
sudo cp 99-xvisio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
sudo dpkg -i xvsdk_3.2.0-20240905_focal_amd64.deb
# 请自行下载xvsdk_3.2.0-2_20220913_amd64.snap安装
# sudo snap install --devmode xvsdk_3.2.0-2_20220913_amd64.snap
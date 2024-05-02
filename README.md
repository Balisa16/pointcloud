# Pointcloud (Beta)

Simple App to display several pcd file using OpenGL

## Build 

### Install Dependencies
```
sudo apt update && sudo apt upgrade
sudo apt install libpcl-dev libboost-all-dev libeigen3-dev libwayland-dev libxkbcommon-dev
```

### Clone and Build
```
git clone https://github.com/Balisa16/pointcloud.git
cd pointcloud
git submodule update --init
mkdir -p build sample && cd build
cmake .. && make -j$(( $(nproc) - 1 ))
```
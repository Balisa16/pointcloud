# Pointcloud (Beta)

Simple App to display several pcd file using OpenGL

## Build 

### Install Dependencies
```
sudo apt update && sudo apt upgrade
sudo apt install libpcl-dev libboost-all-dev libeigen3-dev
```

### Clone and Build
```
git clone https://github.com/Balisa16/pointcloud.git
cd pointcloud
mkdir -p build && cd build
cmake .. && make -j$(( $(nproc) - 1 ))
```
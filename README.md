# visual_slam

### Description

Visual SLAM example for beginner

##### `Currently, only visual odometry has been implemented` 

### Dependency

- Ubuntu 22.04
- OpenCV 4.8.1
- Eigen : 3.3.8
- Sophus : 1.22.10
- glew : 2.1.0
- Pangolin v0.6
- g2o

#### if you use docker, build dockerfile
```bash
git clone https://github.com/ai-robotics-kr/2024-visual-slam.git
cd visual_slam
docker build -t slam .
```

### Docker run
```bash
docker run -it -d --name=slam --privileged --network=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY=$DISPLAY --shm-size=8G --ipc=host -v /dev/shm:/dev/shm slam
```

### How to Usage in local
```bash
git clone https://github.com/ai-robotics-kr/2024-visual-slam.git
cd visual_slam
mkdir build
cd build
cmake ..
make
../bin/MAIN
```

### Reference
[slambook2 : https://github.com/gaoxiang12/slambook2](https://github.com/gaoxiang12/slambook2)
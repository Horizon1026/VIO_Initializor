# VIO_Initializor
Several initializor of visual-inertial odometry.

# Components
- [x] Data manager.
    - [x] Local map.
    - [x] Visualizor.
- [x] Data loader.
- [x] Frontend.
    - [x] Stereo visual frontend.
- [x] Backend.
    - [x] Initialization. (VINS-Mono)
    - [x] Initialization. (DRT-VIO-Init)
    - [ ] Initialization. (OpenVINS)
    - [ ] Initialization. (ORB-SLAM3)

# Dependence
- Slam_Utility
- Feature_Detector
- Feature_Tracker
- Sensor_Model
- Vision_Geometry
- Image_Processor
- Slam_Solver
- Visual_Frontend
- Binary_Data_Log
- Visualizor2D
- Visualizor3D

# Compile and Run
- 第三方仓库的话需要自行 apt-get install 安装
- 拉取 Dependence 中的源码，在当前 repo 中创建 build 文件夹，执行标准 cmake 过程即可
```bash
mkdir build
cmake ..
make -j
```
- 编译成功的可执行文件就在 build 中，具体有哪些可执行文件可参考 run.sh 中的列举。可以直接运行 run.sh 来依次执行所有可执行文件

```bash
sh run.sh
```

# Tips
- 欢迎一起交流学习，不同意商用；
- 这是为了学习开源的 VIO 初始化而创建的仓库，可以切换不同的 VIO 初始化方法
- 通过在 /src/CMakeLists.txt 中的 #Line16 切换不同的初始化方法

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

# Tips
- 欢迎一起交流学习，不同意商用；
- 这是为了学习开源的 VIO 初始化而创建的仓库，可以切换不同的 VIO 初始化方法
- 通过在 /src/CMakeLists.txt 中的 #Line16 切换不同的初始化方法

# VIO_Stereo_SchurVINS
A simple visual-inertial odometry reconstructed from schur-vins.

# Components
- [ ] Data manager.
    - [ ] Local map.
    - [ ] Visualizor.
- [ ] Data loader.
- [ ] Frontend.
    - [ ] Stereo visual frontend.
- [ ] Backend.
    - [ ] Backend initialization.
    - [ ] Backend propagation.
    - [ ] Backend update.
- [ ] Log record.

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
- 这是为了学习开源的 SchurVINS 而创建的用于复现/魔改 paper 的代码仓库，欢迎一起交流学习，不同意商用；
- VINS 的初始化是 OpenVINS 的初始化方法；
- 依赖仓库为各个“积木”仓库，陈列在本文 Dependence 中。如需编译（标准 cmake 编译流程），需要拉下所有依赖仓库的源码。如需运行，需要更改 run.sh 中的数据集路径，在 test_vio.cpp 中配置参数，并在各个“积木”仓库的同一路径下创建 ./Slam_Workspace/output/ 文件夹，用于保存输出文件；
- 不依赖 opencv 和开源求解器（如 ceres，g2o，gtsam之类），是自己实现的求解器，详见 Slam_Solver 仓库；
- Pipeline 暂时没有时间整理成文档，可以参考 vio.RunOnce() 的流程，以及 frontend.RunOnce() 和 backend.RunOnce()；

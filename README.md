# Exploiting Rotational Symmetry for Accurate Object Pose Estimation from Point Clouds without Known 3D Models

This project provides the official implementation of 'Exploiting Rotational Symmetry for Accurate Object Pose Estimation from Point Clouds without Known 3D Models'.
Our research will be published in *The Visual Computer*, a leading journal in computer vision and graphics.

## Abstract

Accurate object pose estimation from point clouds is crucial for various industrial applications, such as automated robotic spray painting. However, the confidential nature of industrial objects often limits access to high-quality 3D models, posing a significant challenge for pose estimation. In this study, we propose a novel method that exploits the rotational symmetry commonly found in industrial objects to address this issue. Our approach jointly estimates the object pose and refines the point cloud in an iterative manner, relying on the constraint of rotational symmetry. By rotating each 3D point based on the currently estimated pose and identifying correspondences using nearest neighbor search, we compute the rotational symmetry constraint loss to iteratively refine both the pose and point cloud. The proposed method achieves robust pose estimation and generalizes well across diverse object types. Experimental results on a dataset comprising synthetic objects and a real wheel hub demonstrate that our method performs comparably to methods relying on known 3D models, achieving an average ADD-S of 0.031 for wheel hubs, 0.017 for impellers, 0.013 for small parts, and 0.036 for propellers.

## Environment

Our code use conda environment
First, run the following commands to set up the conda environment:
```Shell
conda env create -f env.yaml
conda activate RSPoseEstimation
```
Then,our code relies on the ceres-solver open-source library. Install it using:
```Shell
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev
```
Download ceres-solver:
* [ceres-solver](http://ceres-solver.org/ceres-solver-2.2.0.tar.gz)
```Shell
tar zxf ceres-solver-2.2.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.2.0
make -j3
make test
# Optionally install Ceres, it can also be exported using CMake which
# allows Ceres to be used without requiring installation, see the documentation
# for the EXPORT_BUILD_DIR option for more information.
make install
```
Finally, install nanoflann:
```Shell
sudo apt install libnanoflann-dev
```

## Code structure
```shell
├── core
│   ├── io_pc.h
│   ├── median.h
│   ├── pose_optimization.cc
│   ├── pose_point_optimization.cc
│   └── utils.h
├── CursePoseOptimize.py
├── DeleteBottom.py
├── calMax.py
├── eval.py
├── EvalTP.sh
├── generatePointCloudFromDepth.py
├── MixPose.py
├── PointFilter.py
├── PredictedPose_TandPoints.sh
├── PredictedPose_T.sh
└── utils
    ├── ADD_util.py
    ├── camera_util.py
    ├── DepthFilterUtil.py
    ├── FilterUtil.py
    ├── PC_center_normal_utils.py
    ├── PlyUtil.py
    ├── pose_utils.py
    └── SurfaceFinder.py
```
The `core` directory houses the core implementation of the algorithm. Specifically, `pose_optimization.cc` focuses on pose optimization, while `pose_point_optimization.cc` performs joint optimization of both pose and point cloud. The `utils` folder in the root directory contains utility classes for preprocessing steps. The algorithm workflow is initiated by executing `bash scripts` that invoke the remaining Python files.
## Datasets
The wheel hub dataset is provided by the manufacturer and is available for non-commercial use. If you require access for other purposes, please submit a formal request to us.
## Demo
optimize pose:
```Shell
PredictedPose_T.sh
```
optimize both pose and points:
```Shell
PredictedPose_TandPoints.sh
```


## Evaluation

You can evaluate result using `script`:
```Shell
EvalTP.sh
```

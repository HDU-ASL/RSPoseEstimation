# Rotational Symmetry based Object Pose Estimation from Point Clouds in the Absence of Known 3D Models

This project provides the official implementation of 'Rotational Symmetry based Object Pose Estimation from Point Clouds in the Absence of Known 3D Models'.

## Abstract

Object pose estimation is crucial to many industrial applications, with one example being the automated spray painting with a robot. However, confidentiality concerns often limit access to high-quality 3D models, posing a significant challenge for pose estimation based on point clouds. In such scenarios, rotational symmetry—a readily accessible characteristic of many industrial objects—can provide valuable prior information to facilitate pose estimation. 
In this paper, a method is proposed to leverage the rotational symmetry commonly found in industrial objects to address the challenge caused by the absence of 3D models. 
The pose of the object is jointly estimated with point cloud refinement in iterations.  This iterative optimization relies on the loss of the constraint of rotational symmetry. To construct such loss of rotational symmetry, each 3D point is rotated based on the currently estimated pose, and multiple correspondences are then identified using the nearest neighbor search through exploiting the property of rotational symmetry. 
The identified correspondences are used to compute the rotational symmetry constraint loss, which iteratively refines both the pose and the point cloud. By explicitly incorporating rotational symmetry into the optimization process, the proposed method achieves robust pose estimation and generalizes well across diverse object types.
The proposed method is evaluated using data specifically created from point clouds without known 3D models, which comprises a dataset from four types of synthetic objects and one real wheel hub from the production line. 
Experimental results demonstrate that the proposed method achieves a performance comparable to methods relying on known 3D models.

## Environment

Our code use conda environment
First, run the following commands to set up the conda environment:
```Shell
conda env create -f env.yaml
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
## Datasets
The wheel hub dataset is provided by the manufacturer and is available for non-commercial use. If you require access for other purposes, please submit a formal request.
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

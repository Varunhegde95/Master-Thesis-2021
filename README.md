# Master-Thesis-2021
[![Docker Builds](https://github.com/LYON-WANG/Master-Thesis-2021/actions/workflows/main.yml/badge.svg)](https://github.com/LYON-WANG/Master-Thesis-2021/actions/workflows/main.yml)

## opendlv-kitti-replay:

Replay the KITTI data package. Output: IMU, GPS, Lidar

## opendlv-unscented-kalman-filter:

Use UKF to estimate vehicle state.

## opendlv-map-plotter:

Plot the filtered vehicle trajectory.

## opendlv-pointcloud-preprocessing:

Use PCL to process the point cloud. The filtered pointcloud will be used for future registration.
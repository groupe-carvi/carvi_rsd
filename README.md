# carvi_rsd
A Rust wrapper for Robosense rs_driver

This crate provides Rust bindings for the Robosense rs_driver C++ library, enabling seamless integration of Robosense LiDAR devices into Rust applications. It leverages the power of Rust's safety and concurrency features while utilizing the robust capabilities of the rs_driver library.

> [!WARNING]
> This crate is still in early development and is used primarily for CARVI's internal projects and as such may not implement all features of the underlying C++ library. You are welcome to contribute or open issues for missing features.
> At this time, we only tested with Robosense Airy LiDAR.

## Build Instructions
This crate depends on the [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver) repository. And so you need to make sure that you install rs_driver dependency before building this crate.

```sh
sudo apt-get install libpcap-dev libeigen3-dev libboost-dev libpcl-dev
```

# opendlv-unscented-kalman-filter

## Overview

## **Unscented Kalman FIlter (UKF):** 

### Input:  

	* IMU
	* GPS.

### Output: ```opendlv::fused::Movement ```

	* Position X, Y, Z
	* Speed.

### Use (docker-compose):

```
unscented-kalman-filter:
        container_name: unscented-kalman-filter
        build:
            context: ./opendlv-unscented-kalman-filter
            dockerfile: Dockerfile
        image: opendlv-unscented-kalman-filter:v0.1
        ipc: "host"
        network_mode: "host"
        depends_on:
        - "kitti-replay"
        command: "--cid=111 --freq=10 --imu-freq=10 --gps-freq=10 --verbose"

```

### UKF Explanation:

#### 1. States:




# IMU-TK

This is a ROS wrapper for imu_tk, learn details from original [README.md](./README_original.md) please.

## Build

Place this package inside your catkin workspace (e.g. `~/catkin_ws/src`), then build it through `catkin_make`.

```Bash
cd ~/catkin_ws
catkin_make
rospack profile
source devel/setup.bash
```

## Test

### Collect IMU Data

Record a bag file with IMU topic (e.g. `rosbag record -O imu /imu`).

Procedure:

1. Left the IMU static for 50 seconds.
2. Rotate the IMU and then lay it in a different attitude.
3. Wait for at least 1 seconds.
4. Have you rotated the IMU 36 ~ 50 times? If not, go back to step 2.
5. Done.

### Calibration

Run `imu_calib_node` to get calibration result(e.g. misalignment, scale and bias).

`imu_calib_node` usage: 

> rosrun imu_tk imu_calib_node [BAG] [IMU_TOPIC]

```Bash
rosrun imu_tk imu_calib_node imu.bag /imu
```

### Correction

Given a raw sensor reading X (e.g., the acceleration ), the calibrated "unbiased" reading X' is obtained.

```Txt
Misalignment matrix:

    [    1     -mis_yz   mis_zy  ]
T = [  mis_xz     1     -mis_zx  ]
    [ -mis_xy   mis_yx     1     ]

Scale matrix:

    [  s_x      0        0  ]
K = [   0      s_y       0  ]
    [   0       0       s_z ]

Bias vector:

    [ b_x ]
B = [ b_y ]
    [ b_z ]

X' = T*K*(X - B)
```
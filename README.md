# imu_driver_node
This is driver node I wrote for the Sparton AHRS-8 IMU. This driver has speed improvements compared to the Python driver we originally had, but still desyncs with the GPS after a short while.

This driver should be included in **~/ninebits/carson_ws/src** on the NUC with the RTK code. The launch file there already launches this code when it is included.

[The main source file](./src/driver_node/src/compass.cpp) needs to be modified to take the path to the IMU as an argument instead of being hardcoded.

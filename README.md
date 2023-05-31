# ros2_perf_test

Package to run a simple ROS2 pub/sub performance test

## How to install

Clone into your ROS2 workspace and build:
```
mkdir -p ~/ws/src
cd ~/ws/src
git clone git@github.com:berndpfrommer/ros2_perf_test.git
```

## How to run performance tests under ROS2 using fastrtps DDS
Prepare:
```
cd ~/ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
. install/setup.bash
```
Point to fastrtps config file:
```
cd ~/ws/src/ros2_perf_test
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=`pwd`/FastRTPSProfiles.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
```
Run publisher in one terminal (for standard message tests):
```
ros2 run ros2_perf_test publisher_node -r 40.0 -s 1000000 -q 1
```

Run subscriber in another terminal (separated by slow link/wifi!):
```
ros2 run ros2_perf_test subscriber_node
```

The moment the subscriber connects, the publisher rate drops:
```
[INFO] [1676144437.578775442] [test_publisher]: size: 1000000 rate: 39.99 bw:  319.944 Mbits/s
[INFO] [1676144438.628021027] [test_publisher]: size: 1000000 rate: 30.50 bw:  243.987 Mbits/s
[INFO] [1676144439.673727937] [test_publisher]: size: 1000000 rate: 15.30 bw:  122.402 Mbits/s
[INFO] [1676144440.740182488] [test_publisher]: size: 1000000 rate: 13.13 bw:  105.023 Mbits/s
```

## How to achieve nonblocking publish under ROS2:
Unfortunately this is only possible under FASTRTPS. No other DDS seems
to offer this option.
```
cd src/ros2_perf_test
export FASTRTPS_DEFAULT_PROFILES_FILE=`pwd`/FastRTPSProfiles.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

#
## License

This software is issued under the Apache License Version 2.0.

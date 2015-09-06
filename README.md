# Cross Road Detection
## Brief introduction
Detect the position of crossroad using /in2_msgs/ScanInfoV2 formed data. 

## Step by Step
### step1
Install ROS first, creat and init your ros workspace

```cd ~```

```mkdir -p catkin_ws/src```

```catkin_init_workspace```

```catkin_create_pkg crossroadet roscpp```

```cd ~/catkin_ws```

```catkin_make```

```rosrun roadrec distmap```

### step2

```cd ~```

```mkdir myCrossRoadDetRepo```

```git init```

### step3

```git clone https://github.com/qiterbit/crossroadet.git```


# BodyTrackerAzure

## Description
This Visual Studio project extracts the 3-dimensional position and orientation data of body joints from an Azure Kinect sensor and forwards it to a ROS system. It features:
1. Extracting joint and IMU data from a Kinect sensor;
1. Transmitting data via rosserial to a computer where ROS serial server is running;
1. Receiving UDP synchronization packets from a sportsole data logger node.


## Prerequisite
1. Windows 10
1. [Azure Kinect Sensor SDK](https://docs.microsoft.com/en-us/azure/Kinect-dk/sensor-sdk-download)
1. [Azure Kinect Body Tracking SDK](https://docs.microsoft.com/en-us/azure/Kinect-dk/body-sdk-download)
1. Visual Studio 2017 (Version 15.7.4 or above)
1. A computer running ROS Kinetic with package [gait_training_robot](https://github.com/zchenpds/gait_training_robot) and a forked [rosserial](https://github.com/ral-stevens/rosserial).

## Cloning
Since it contains a submodule, please use option "--recurse-submodule" to clone to a local directory.
```
git clone --recurse-submodule https://github.com/zchenpds/KinectBodyTracker
```

## Compiling the source
1. Before compiling the source, you might need to retarget the solution if you have a higher version of Windows SDK. To do this, right click on your solution in the Solution Explorer, and then choose "Retarget solution". 
1. The target platform has to be x64. For `Release`(`Debug`) configuration, the target files will be generated in `.\x64\Release`(`.\x64\Debug`). Upon startup of `.exe` program, the `config.txt` file in the same directory as the `.exe` program will be loaded.

## Parameter Configuration
- `ros_master=192.168.0.101:11411`: IP and port number of the rosserial server.
- `RosSocket/skeletonPub/enabled=false`: Publish the whole skeleton or not. The pelvis position will be published regardless of this parameter.
- `RosSocket/imuPub/enabled=false`: Publish IMU messages or not.
- `RosSocket/timeout_ms=3000`: (Obsolete)
- `k4a/depth_mode=3`: The value ranges from 0 to 5, each correponding to one of the enumeration values defined [here](https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/group___enumerations_ga3507ee60c1ffe1909096e2080dd2a05d.html#ga3507ee60c1ffe1909096e2080dd2a05d)
- `CsvLogger/enabled=true`
- `CsvLogger/dataPath=.\..\..\data`: The path where the csv files will be saved at.

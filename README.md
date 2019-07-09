# KinectBodyTracker4

## Description
This Visual Studio project extracts the positional data of body joints from an Azure Kinect sensor and forwards it to a ROS system. It features:
1. Extracting joint and IMU data from a Kinect sensor;
1. Transmitting data via rosserial to a Jetson TX2 computer running ROS;
1. Receiving UDP synchronization packets from the Jetson TX2 computer, one packet per second.


## Prerequisite
1. Windows 10
1. [Azure Kinect Sensor SDK](https://docs.microsoft.com/en-us/azure/Kinect-dk/sensor-sdk-download)
1. [Azure Kinect Body Tracking SDK](https://docs.microsoft.com/en-us/azure/Kinect-dk/body-sdk-download)
1. Visual Studio 2017 (Version 15.7.4 or above)

## Cloning
Since it contains a submodule, please use option "--recurse-submodule" to clone to a local directory.
```
git clone --recurse-submodule https://github.com/zchenpds/KinectBodyTracker
```

## Compiling the source
1. Before compiling the source, you might need to retarget the solution if you have a higher version of Windows SDK. To do this, right click on your solution in the Solution Explorer, and then choose "Retarget solution". 
1. Once the source code is compiled, a `.exe` file will be generated in `.\x64\Debug`.

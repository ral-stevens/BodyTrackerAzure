#pragma once
#include <stdio.h>
#include "Config.h"
#include <thread>
#include <mutex>
#include <chrono>
#include <array>
#include "gait_training_robot/HumanSkeletonAzure.h"
#include "gait_training_robot/ImuAzure.h"
#include "rosserial_windows/ros_lib/sensor_msgs/Imu.h"
#include "rosserial_windows/ros_lib/tf/transform_broadcaster.h"
#include "rosserial_windows/ros_lib/geometry_msgs/TransformStamped.h"
#include "include/tf2/LinearMath/Quaternion.h"
#include "include/tf2/LinearMath/Matrix3x3.h"


enum RosSocketStatus_t
{
	RSS_Failed = 0,
	RSS_Connecting,
	RSS_Connected
};

class RosSocket
{
private:
	std::string m_strCameraBaseFrame = "kinect_azure/camera_base";
	//std::string m_strRgbFrame = "kinect_azure/rgb_camera_link";
	std::string m_strDepthFrame = "kinect_azure/depth_camera_link";
	std::string m_strImuFrame = "kinect_azure/imu_link";

	std::string m_strSkeletonTopic = "/skeleton";
	std::string m_strImuTopic = "/kinect_azure_imu";
public:
	RosSocket();
	~RosSocket();
	void setStatusUpdatingFun(std::function<void(static_control_type, const wchar_t*)> fun);
	void updateStatus();
	RosSocketStatus_t getStatus();
	void threadProc();
	void publishMsgSkeleton(const k4abt_skeleton_t & skeleton, uint32_t id, uint64_t k4a_timestamp_usec);
	void publishMsgImu(const k4a_imu_sample_t & imu_sample);
	
	// reference: https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/src/k4a_calibration_transform_data.cpp
	void broadcastDepthTf(const k4a_calibration_t * k4a_calibration);
	void broadcastImuTf(const k4a_calibration_t * k4a_calibration);
	ros::Time timestampToROS(const uint64_t & k4a_timestamp_us);
private:
	ros::NodeHandle			nh;
	std::string				m_strRosMaster;
	RosSocketStatus_t		m_nStatus;
	bool                    m_bTerminating;
	INT64                   m_nLastUpdateTime;
	//int                     m_nTimeout_ms;
	uint64_t				m_nSpinCounter;

	// Status update
	std::wstring            m_WstrStatusMessage;
	std::function<void(static_control_type, const wchar_t *)>   m_funPrintMessage;
	

	// Human joint messages
	gait_training_robot::HumanSkeletonAzure	m_MsgSkeleton;
	//gait_training_robot::ImuAzure           m_MsgIMU;
	sensor_msgs::Imu                        m_MsgIMU;
	ros::Publisher			                m_PubSkeleton;
	ros::Publisher			                m_PubIMU;
	std::array<tf::TransformBroadcaster, 3> m_TfBroadcasters;
	ros::Time				m_tStartTime;

	std::mutex              m_Mutex;
	std::thread             m_Thread;
};



// Publish a TF link so the URDF model and the depth camera line up correctly
#define DEPTH_CAMERA_OFFSET_MM_X 0.0f
#define DEPTH_CAMERA_OFFSET_MM_Y 0.0f
#define DEPTH_CAMERA_OFFSET_MM_Z 1.8f // The depth camera is shifted 1.8mm up in the depth window

tf2::Vector3 getDepthToBaseTranslationCorrection();

tf2::Quaternion getDepthToBaseRotationCorrection(const k4a_calibration_t * k4a_calibration);
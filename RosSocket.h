#pragma once
#include <stdio.h>
#include "Config.h"
#include <thread>
#include <mutex>
#include <chrono>
#include "gait_training_robot/HumanSkeletonAzure.h"
#include "gait_training_robot/ImuAzure.h"

enum RosSocketStatus_t
{
	RSS_Failed = 0,
	RSS_Connecting,
	RSS_Connected
};

class RosSocket
{
public:
	RosSocket();
	~RosSocket();
	void setStatusUpdatingFun(std::function<void(static_control_type, const wchar_t*)> fun);
	void updateStatus();
	RosSocketStatus_t getStatus();
	void threadProc();
	void publishMsgSkeleton(const k4abt_skeleton_t & skeleton, uint32_t id, uint64_t k4a_timestamp_usec);
	void publishMsgImu(const k4a_imu_sample_t & imu_sample);
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
	gait_training_robot::ImuAzure           m_MsgIMU;
	ros::Publisher			m_PubSkeleton;
	ros::Publisher			m_PubIMU;

	std::mutex              m_Mutex;
	std::thread             m_Thread;
};


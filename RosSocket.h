#pragma once
#include <stdio.h>
#include "Config.h"
#include <thread>
#include <chrono>
#include "gait_training_robot/human_skeleton.h"

enum RosSocketStatus_t
{
	RSS_Failed = 0,
	RSS_Timeout,
	RSS_Connecting,
	RSS_Connected,
};

class RosSocket
{
public:
	RosSocket();
	~RosSocket();
	void setStatusUpdatingFun(std::function<void(const wchar_t *)> fun);
	RosSocketStatus_t getStatus();
	void threadProc();
	void publishMsgSkeleton(const k4abt_skeleton_t & skeleton);
private:
	ros::NodeHandle			nh;
	std::string				m_strRosMaster;
	RosSocketStatus_t		m_nStatus;
	INT64                   m_nLastUpdateTime;
	uint64_t				m_nSpinCounter;
	char*                   m_pszRosMaster;

	// Status update
	std::wstring            m_WstrStatusMessage;
	std::function<void(const wchar_t *)>   m_funUpdateStatus;
	

	// Human joint messages
	gait_training_robot::human_skeleton	m_MsgSkeleton;
	ros::Publisher			m_PubSkeleton;
	std::thread				m_Thread;
	
};


#include "stdafx.h"
#include "RosSocket.h"
#include <string>


RosSocket::RosSocket():
	m_strRosMaster("192.168.1.116:11411"),
	m_nStatus(RSS_Connecting),
	m_nLastUpdateTime(GetTickCount64()),
	m_nSpinCounter(0),
	m_pszRosMaster(NULL),
	m_PubSkeleton("/skeleton", &m_MsgSkeleton),
	m_Thread(&RosSocket::threadProc, this)
{	
	m_Thread.detach();
}


RosSocket::~RosSocket()
{
	delete[] m_pszRosMaster;
}

void RosSocket::setStatusUpdatingFun(std::function<void(const wchar_t*)> fun)
{
	m_funUpdateStatus = fun;
}

RosSocketStatus_t RosSocket::getStatus()
{
	if (nh.connected()) {
		m_nStatus = RSS_Connected;
		m_WstrStatusMessage = std::wstring(L"Connected to rosserial server at ") +
			std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(m_strRosMaster);;
		if (m_funUpdateStatus) m_funUpdateStatus(m_WstrStatusMessage.c_str());
	}
	else if (m_nSpinCounter > 10) {
		m_nStatus = RSS_Failed;
		m_WstrStatusMessage = std::wstring(L"Disconnected from rosserial server at ") +
			std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(m_strRosMaster);;
		if (m_funUpdateStatus) m_funUpdateStatus(m_WstrStatusMessage.c_str());
	}
		
	int timeout_ms;
	Config::Instance()->assign("RosSocket/timeout_ms", timeout_ms);
	if (GetTickCount64() - m_nLastUpdateTime > timeout_ms) {
		m_nStatus = RSS_Timeout;
		m_WstrStatusMessage = std::wstring(L"Connecting to rosserial server timed out.");
		if (m_funUpdateStatus) m_funUpdateStatus(m_WstrStatusMessage.c_str());
	}
	
	return m_nStatus;
}


void RosSocket::threadProc()
{
	Config* pConfig = Config::Instance();
	pConfig->assign("ros_master", m_strRosMaster);
	std::unique_ptr<char[]> m_pszRosMaster(new char[m_strRosMaster.length() + 1]);
	std::strcpy(m_pszRosMaster.get(), m_strRosMaster.c_str());

	m_WstrStatusMessage = std::wstring(L"Connecting to rosserial server at ") + 
		std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(m_strRosMaster);
	if (m_funUpdateStatus) m_funUpdateStatus(m_WstrStatusMessage.c_str());
	//printf("Connecting to server at %s\n", ros_master);
	nh.initNode(m_pszRosMaster.get());

	m_MsgSkeleton.header.frame_id = "/kinect2_link";
	m_MsgSkeleton.header.seq = 0;
	m_MsgSkeleton.point_length = K4ABT_JOINT_COUNT;
	nh.advertise(m_PubSkeleton);

	while (1) {
		// Spin
		nh.spinOnce();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		m_nLastUpdateTime = GetTickCount64();
		m_nSpinCounter++;
		if (getStatus() == RSS_Failed) {
			break;
		}
	}

	
}

void RosSocket::publishMsgSkeleton(const k4abt_skeleton_t & skeleton)
{
	m_MsgSkeleton.header.seq++;
	m_MsgSkeleton.header.stamp = nh.now();

	geometry_msgs::Point jointPoints[K4ABT_JOINT_COUNT];
	for (int i = 0; i < K4ABT_JOINT_COUNT; i++) {
		jointPoints[i].x = skeleton.joints[i].position.xyz.x;
		jointPoints[i].y = skeleton.joints[i].position.xyz.y;
		jointPoints[i].z = skeleton.joints[i].position.xyz.z;
	}

	m_MsgSkeleton.point = jointPoints;
	m_PubSkeleton.publish(&m_MsgSkeleton);
}

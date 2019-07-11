#include "stdafx.h"
#include "RosSocket.h"
#include <string>


RosSocket::RosSocket() :
	m_strRosMaster("192.168.1.116:11411"),
	m_nStatus(RSS_Connecting),
	m_bTerminating(false),
	m_nLastUpdateTime(GetTickCount64()),
	//m_nTimeout_ms(2000),
	m_nSpinCounter(0),
	m_PubSkeleton("/skeleton", &m_MsgSkeleton),
	m_PubIMU("/kinect_azure_imu", &m_MsgIMU),
	m_Thread(&RosSocket::threadProc, this)
{	
	
}


RosSocket::~RosSocket()
{
	m_bTerminating = true;
	m_Thread.join();
}

void RosSocket::setStatusUpdatingFun(std::function<void(const wchar_t*)> fun)
{
	m_funUpdateStatus = fun;
}

void RosSocket::updateStatus()
{
	std::lock_guard<std::mutex> lk(m_Mutex);
	if (nh.connected()) {
		m_nStatus = RSS_Connected;
		// Ensure the message printing interval to be larger than 500 ms.
		static INT64 timePrev = GetTickCount64();
		if (GetTickCount64() - timePrev > 500)
		{
			m_WstrStatusMessage = std::wstring(L"Connected to rosserial server at ") +
				std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(m_strRosMaster);
			if (m_funUpdateStatus) m_funUpdateStatus(m_WstrStatusMessage.c_str());
			timePrev = GetTickCount64();
		}
	}
	else if (m_nSpinCounter > 2) {
		m_nStatus = RSS_Failed;
		m_WstrStatusMessage = std::wstring(L"Disconnected from rosserial server at ") +
			std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(m_strRosMaster);
		if (m_funUpdateStatus) m_funUpdateStatus(m_WstrStatusMessage.c_str());
	}
}

RosSocketStatus_t RosSocket::getStatus()
{
	return m_nStatus;
}

void RosSocket::threadProc()
{
	Config* pConfig = Config::Instance();
	pConfig->assign("ros_master", m_strRosMaster);
	//pConfig->assign("RosSocket/timeout_ms", m_nTimeout_ms);
	std::unique_ptr<char[]> pszRosMaster(new char[m_strRosMaster.length() + 1]);
	std::strcpy(pszRosMaster.get(), m_strRosMaster.c_str());

	m_WstrStatusMessage = std::wstring(L"Connecting to rosserial server at ") + 
		std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(m_strRosMaster);
	if (m_funUpdateStatus) m_funUpdateStatus(m_WstrStatusMessage.c_str());
	//printf("Connecting to server at %s\n", ros_master);
	nh.initNode(pszRosMaster.get());

	// Prepare for publishing skeleton data
	m_MsgSkeleton.header.frame_id = "/kinect_azure_link";
	m_MsgSkeleton.header.seq = 0;
	nh.advertise(m_PubSkeleton);

	// Prepare for publishing IMU data
	m_MsgIMU.header.frame_id = "/kinect_azure_link";
	m_MsgIMU.header.seq = 0;
	nh.advertise(m_PubIMU);

	while (updateStatus(), !(getStatus() == RSS_Failed || m_bTerminating)) {
		// Spin
		nh.spinOnce();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		m_nLastUpdateTime = GetTickCount64();
		m_nSpinCounter++;
	}
	
}

void RosSocket::publishMsgSkeleton(const k4abt_skeleton_t & skeleton, uint32_t id, uint64_t k4a_timestamp_usec)
{
	m_MsgSkeleton.header.seq++;
	m_MsgSkeleton.header.stamp = nh.now();
	m_MsgSkeleton.id = id;
	m_MsgSkeleton.k4a_timestamp_usec = k4a_timestamp_usec;

	geometry_msgs::Point jointPoints[K4ABT_JOINT_COUNT];
	for (int i = 0; i < K4ABT_JOINT_COUNT; i++) {
		geometry_msgs::Point & position_lhs = m_MsgSkeleton.poses[i].position;
		const k4a_float3_t & position_rhs = skeleton.joints[i].position;
		position_lhs.x = position_rhs.xyz.x;
		position_lhs.y = position_rhs.xyz.y;
		position_lhs.z = position_rhs.xyz.z;

		geometry_msgs::Quaternion & orientation_lhs = m_MsgSkeleton.poses[i].orientation;
		const k4a_quaternion_t & orientation_rhs = skeleton.joints[i].orientation;
		orientation_lhs.x = orientation_rhs.wxyz.x;
		orientation_lhs.y = orientation_rhs.wxyz.y;
		orientation_lhs.z = orientation_rhs.wxyz.z;
		orientation_lhs.w = orientation_rhs.wxyz.w;
	}

	m_PubSkeleton.publish(&m_MsgSkeleton);
}

void RosSocket::publishMsgImu(const k4a_imu_sample_t & imu_sample)
{
	m_MsgIMU.header.seq++;
	m_MsgIMU.header.stamp = nh.now();

	m_MsgIMU.angular_velocity.x = imu_sample.gyro_sample.xyz.x;
	m_MsgIMU.angular_velocity.y = imu_sample.gyro_sample.xyz.y;
	m_MsgIMU.angular_velocity.z = imu_sample.gyro_sample.xyz.z;
	m_MsgIMU.gyro_timestamp_usec = imu_sample.gyro_timestamp_usec;

	m_MsgIMU.linear_acceleration.x = imu_sample.acc_sample.xyz.x;
	m_MsgIMU.linear_acceleration.y = imu_sample.acc_sample.xyz.y;
	m_MsgIMU.linear_acceleration.z = imu_sample.acc_sample.xyz.z;
	m_MsgIMU.acc_timestamp_usec = imu_sample.acc_timestamp_usec;

	m_PubIMU.publish(&m_MsgIMU);
}

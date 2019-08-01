#include "stdafx.h"
#include "RosSocket.h"
#include <string>
#include "CsvLogger.h"
#include <sstream>

RosSocket::RosSocket() :
	m_strRosMaster("192.168.1.116:11411"),
	m_nStatus(RSS_Connecting),
	m_bTerminating(false),
	m_nLastUpdateTime(GetTickCount64()),
	//m_nTimeout_ms(2000),
	m_nSpinCounter(0),
	m_PubSkeleton(m_strSkeletonTopic.c_str(), &m_MsgSkeleton),
	m_PubIMU(m_strImuTopic.c_str(), &m_MsgIMU),
	m_Thread(&RosSocket::threadProc, this)
{	
	std::for_each(m_TfBroadcasters.begin(), m_TfBroadcasters.end(), 
		[&](tf::TransformBroadcaster & br) {br.init(nh); });
}


RosSocket::~RosSocket()
{
	m_bTerminating = true;
	m_Thread.join();
}

void RosSocket::setStatusUpdatingFun(std::function<void(static_control_type, const wchar_t*)> fun)
{
	m_funPrintMessage = fun;
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
			if (m_funPrintMessage) m_funPrintMessage(SCT_RosSocket, m_WstrStatusMessage.c_str());
			timePrev = GetTickCount64();
		}
	}
	else if (m_nSpinCounter > 2) {
		m_nStatus = RSS_Failed;
		m_WstrStatusMessage = std::wstring(L"Disconnected from rosserial server at ") +
			std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(m_strRosMaster);
		if (m_funPrintMessage) m_funPrintMessage(SCT_RosSocket, m_WstrStatusMessage.c_str());
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
	if (m_funPrintMessage) m_funPrintMessage(SCT_RosSocket, m_WstrStatusMessage.c_str());
	//printf("Connecting to server at %s\n", ros_master);
	nh.initNode(pszRosMaster.get());

	// Prepare for publishing skeleton data
	m_MsgSkeleton.header.frame_id = m_strDepthFrame.c_str();
	m_MsgSkeleton.header.seq = 0;
	nh.advertise(m_PubSkeleton);

	// Prepare for publishing IMU data
	m_MsgIMU.header.frame_id = m_strImuFrame.c_str();
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
	std::wstringstream wss;
	const k4abt_joint_t & pelvis = skeleton.joints[K4ABT_JOINT_PELVIS];
	const float & px = pelvis.position.xyz.x;
	const float & py = pelvis.position.xyz.y;
	const float & pz = pelvis.position.xyz.z;
	
	const float & qw = pelvis.orientation.wxyz.w;
	const float & qx = pelvis.orientation.wxyz.x;
	const float & qy = pelvis.orientation.wxyz.y;
	const float & qz = pelvis.orientation.wxyz.z;

	// Sometimes, the quaternion has four zero components. If so, discard this skeleton.
	if ((qw * qw + qx * qx + qy * qy + qz * qz) < 0.8) return;

	// Broadcast transform
	static geometry_msgs::TransformStamped transform_stamped;
	transform_stamped.header.stamp = nh.now();
	transform_stamped.header.frame_id = m_strDepthFrame.c_str();
	transform_stamped.header.seq++;
	transform_stamped.child_frame_id = "skeleton_pelvis_link";

	transform_stamped.transform.translation.x = px;
	transform_stamped.transform.translation.y = py;
	transform_stamped.transform.translation.z = pz;
	transform_stamped.transform.rotation.w = qw;
	transform_stamped.transform.rotation.x = qx;
	transform_stamped.transform.rotation.y = qy;
	transform_stamped.transform.rotation.z = qz;
	m_TfBroadcasters[0].sendTransform(transform_stamped);
	wss << L"Published pelvis tf with seq = " << transform_stamped.header.seq << L". ";

	bool bSkeletonPubEnabled = false;
	Config::Instance()->assign("RosSocket/skeletonPub/enabled", bSkeletonPubEnabled);
	if (bSkeletonPubEnabled) 
	{
		uint64_t ros_ts = static_cast<uint64_t>(transform_stamped.header.stamp.toNsec());
		uint64_t win_ts = GetTickCount64();
		static CsvLogger logger("pelvis_pose", vector_header_value_t{
			{"ros_ts_nsec", &ros_ts},
			{"win_ts_msec", &win_ts},
			{"k4a_ts_usec", &k4a_timestamp_usec},
			{"px", &px}, {"py", &py}, {"pz", &pz}, // position
			{"qw", &qw}, {"qx", &qx}, {"qy", &qy}, {"qz", &qz} // orientation
			});
		logger.log();

		// Prepare skeleton message to be published
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
		wss << L"Published skeleton msg with seq = " << m_MsgSkeleton.header.seq;
	}
	else
	{
		wss << L"Skeleton msg publisher disabled.";
	}
	if (m_funPrintMessage) m_funPrintMessage(SCT_RosSocket_Skeleton, wss.str().c_str());
}

void RosSocket::publishMsgImu(const k4a_imu_sample_t & imu_sample)
{
	std::wstringstream wss;
	bool bImuPubEnabled = false;
	Config::Instance()->assign("RosSocket/imuPub/enabled", bImuPubEnabled);
	if (bImuPubEnabled)
	{ 
		m_MsgIMU.header.seq++;
		m_MsgIMU.header.stamp = nh.now();

		m_MsgIMU.angular_velocity.x = -1.0 * imu_sample.gyro_sample.xyz.x;
		m_MsgIMU.angular_velocity.y = 1.0 * imu_sample.gyro_sample.xyz.y;
		m_MsgIMU.angular_velocity.z = -1.0 * imu_sample.gyro_sample.xyz.z;
		m_MsgIMU.gyro_timestamp_usec = imu_sample.gyro_timestamp_usec;

		m_MsgIMU.linear_acceleration.x = -1.0 * imu_sample.acc_sample.xyz.x;
		m_MsgIMU.linear_acceleration.y = 1.0 * imu_sample.acc_sample.xyz.y;
		m_MsgIMU.linear_acceleration.z = -1.0 * imu_sample.acc_sample.xyz.z;
		m_MsgIMU.acc_timestamp_usec = imu_sample.acc_timestamp_usec;

		m_PubIMU.publish(&m_MsgIMU);

		uint64_t ros_ts = static_cast<uint64_t>(m_MsgIMU.header.stamp.toNsec());
		uint64_t win_ts = GetTickCount64();
		static CsvLogger logger("imu", vector_header_value_t{
			{"ros_ts_nsec", &ros_ts},
			{"win_ts_msec", &win_ts},
			{"acc_ts_usec", &imu_sample.acc_timestamp_usec},
			{"gyro_ts_usec", &imu_sample.gyro_timestamp_usec},
			{"wx", &imu_sample.gyro_sample.xyz.x},
			{"wy", &imu_sample.gyro_sample.xyz.y},
			{"wz", &imu_sample.gyro_sample.xyz.z},
			{"vx", &imu_sample.acc_sample.xyz.x},
			{"vy", &imu_sample.acc_sample.xyz.y},
			{"vz", &imu_sample.acc_sample.xyz.z}
			});
		logger.log();
		wss << L"Published IMU msg with seq = " << m_MsgIMU.header.seq;
	}
	else
	{
		wss << L"IMU msg publisher disabled.";
	}
	if (m_funPrintMessage) m_funPrintMessage(SCT_RosSocket_IMU, wss.str().c_str());

}

void RosSocket::broadcastDepthTf(const k4a_calibration_t * k4a_calibration)
{
	geometry_msgs::TransformStamped static_transform;

	static_transform.header.stamp = nh.now();
	static_transform.header.frame_id = m_strCameraBaseFrame.c_str();
	static_transform.child_frame_id = m_strDepthFrame.c_str();

	// Translation
	static_transform.transform.translation.x = 0.0 / 1000.0;
	static_transform.transform.translation.y = 0.0 / 1000.0;
	static_transform.transform.translation.z = 1.8 / 1000.0;

	// Rotation
	tf2::Quaternion depth_rotation = getDepthToBaseRotationCorrection(k4a_calibration);

	static_transform.transform.rotation.x = depth_rotation.x();
	static_transform.transform.rotation.y = depth_rotation.y();
	static_transform.transform.rotation.z = depth_rotation.z();
	static_transform.transform.rotation.w = depth_rotation.w();

	m_TfBroadcasters[1].sendTransform(static_transform);
}

void RosSocket::broadcastImuTf(const k4a_calibration_t * k4a_calibration)
{
	k4a_float3_t origin = { 0.0f, 0.0f, 0.0f };
	k4a_float3_t target = { 0.0f, 0.0f, 0.0f };

	k4a_calibration_3d_to_3d(k4a_calibration, &origin,
		K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_ACCEL, &target);

	geometry_msgs::TransformStamped static_transform;

	static_transform.header.stamp = nh.now();
	static_transform.header.frame_id = m_strCameraBaseFrame.c_str();
	static_transform.child_frame_id = m_strImuFrame.c_str();

	tf2::Vector3 extrinsic_translation((target.xyz.x / 1000.0f), (target.xyz.y / -1000.0f), (target.xyz.z / -1000.0f));
	extrinsic_translation += getDepthToBaseTranslationCorrection();

	static_transform.transform.translation.x = extrinsic_translation.x();
	static_transform.transform.translation.y = extrinsic_translation.y();
	static_transform.transform.translation.z = extrinsic_translation.z();

	const k4a_calibration_extrinsics_t * imu_extrinsics = &k4a_calibration->extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_ACCEL];

	tf2::Matrix3x3 imu_matrix(imu_extrinsics->rotation[0], imu_extrinsics->rotation[1], imu_extrinsics->rotation[2],
		imu_extrinsics->rotation[3], imu_extrinsics->rotation[4], imu_extrinsics->rotation[5],
		imu_extrinsics->rotation[6], imu_extrinsics->rotation[7], imu_extrinsics->rotation[8]);

	double yaw, pitch, roll;
	imu_matrix.getEulerYPR(yaw, pitch, roll);

	tf2::Quaternion imu_rotation;
	imu_rotation.setRPY(
		yaw + (90) * M_PI / 180.0,
		roll + (-84) * M_PI / 180.0,   // Rotation around the Y axis must include a 6 degree correction for a static offset collected during calibration
		pitch + (0) * M_PI / 180.0);

	static_transform.transform.rotation.x = imu_rotation.x();
	static_transform.transform.rotation.y = imu_rotation.y();
	static_transform.transform.rotation.z = imu_rotation.z();
	static_transform.transform.rotation.w = imu_rotation.w();

	m_TfBroadcasters[2].sendTransform(static_transform);
}


tf2::Vector3 getDepthToBaseTranslationCorrection()
{
	return tf2::Vector3(DEPTH_CAMERA_OFFSET_MM_X / 1000.0f, DEPTH_CAMERA_OFFSET_MM_Y / 1000.0f, DEPTH_CAMERA_OFFSET_MM_Z / 1000.0f);
}

tf2::Quaternion getDepthToBaseRotationCorrection(const k4a_calibration_t * k4a_calibration)
{
	tf2::Quaternion ros_camera_rotation; // ROS camera co-ordinate system requires rotating the entire camera relative to camera_base
	tf2::Quaternion depth_rotation;      // K4A depth camera has different declinations based on the operating mode (NFOV or WFOV)

	// WFOV modes are rotated down by 7.3 degrees
	if (k4a_calibration->depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED || k4a_calibration->depth_mode == K4A_DEPTH_MODE_WFOV_2X2BINNED)
	{
		depth_rotation.setEuler(0, (-7.3) * M_PI / 180.0, 0);
	}
	// NFOV modes are rotated down by 6 degrees
	else if (k4a_calibration->depth_mode == K4A_DEPTH_MODE_NFOV_UNBINNED || k4a_calibration->depth_mode == K4A_DEPTH_MODE_NFOV_2X2BINNED)
	{
		depth_rotation.setEuler(0, (-6.0) * M_PI / 180.0, 0);
	}
	// Passive IR mode doesn't have a rotation?
	// TODO: verify that passive IR really doesn't have a rotation
	else if (k4a_calibration->depth_mode == K4A_DEPTH_MODE_PASSIVE_IR)
	{
		depth_rotation.setEuler(0, 0, 0);
	}
	else
	{
		throw(std::runtime_error("Could not determine depth camera mode for rotation correction"));
		depth_rotation.setEuler(0, 0, 0);
	}

	ros_camera_rotation.setEuler(M_PI / -2.0f, M_PI, (M_PI / 2.0f));

	return ros_camera_rotation * depth_rotation;
}
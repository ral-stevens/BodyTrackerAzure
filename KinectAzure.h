#pragma once
#include "stdafx.h"
#include "RosSocket.h"


class KinectAzure
{
private:
	k4a_device_t			m_Kinect;
	k4a_device_configuration_t	m_KinectConfig;
	k4a_calibration_t		m_KinectCalibration;
	k4abt_tracker_t			m_KinectBodyTracker;
	k4abt_skeleton_t*		m_pSkeletonClosest;
	bool                    m_bTerminating;
	std::thread             m_ThreadSkeleton;
	std::thread             m_ThreadImu;

	// Status update
	std::wstring            m_WstrStatusMessage;
	std::function<void(const wchar_t *)>   m_funUpdateStatusKinect;
	std::function<void(const wchar_t *)>   m_funUpdateStatusBodyTracker;
	std::function<void(uint64_t nTime, int nBodyCount, const k4abt_skeleton_t *pSkeleton, const uint32_t * pID)>   m_funProcessBody;
	std::function<void(const k4a_imu_sample_t & ImuSample)> m_funProcessIMU;
public:
	KinectAzure(
		std::function<void(const wchar_t*)> funUpdateStatusKinect,
		std::function<void(const wchar_t*)> funUpdateStatusBodyTracker,
		std::function<void(uint64_t nTime, int nBodyCount, const k4abt_skeleton_t *pSkeleton, const uint32_t * pID)> funProcessBody,
		std::function<void(const k4a_imu_sample_t & ImuSample)> funProcessIMU
	);
	
	~KinectAzure();
	void Terminate();
	void SkeletonProc();
	void ImuProc();
	void setParams();
	void EnsureDefaultSensor();
	void ReleaseDefaultSensor();
	void SkeletonUpdate();
	void ImuUpdate();
	const k4a_calibration_t * GetKinectCalibrationPointer();
};


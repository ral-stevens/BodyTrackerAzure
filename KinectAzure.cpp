#include "KinectAzure.h"
#include <chrono>
#include <ctime>
#include <array>

KinectAzure::KinectAzure(
	std::function<void(const wchar_t*)> funUpdateStatusKinect,
	std::function<void(const wchar_t*)> funUpdateStatusBodyTracker,
	std::function<void(uint64_t nTime, int nBodyCount, const k4abt_skeleton_t *pSkeleton, const uint32_t * pID)> funProcessBody,
	std::function<void(const k4a_imu_sample_t & ImuSample)> funProcessIMU
):
	m_Kinect(NULL),
	m_KinectConfig(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL),
	m_KinectBodyTracker(NULL),
	m_pSkeletonClosest(nullptr),
	m_bTerminating(false),
	m_ThreadSkeleton(&KinectAzure::SkeletonProc, this),
	m_ThreadImu(&KinectAzure::ImuProc, this),
	m_funUpdateStatusKinect(funUpdateStatusKinect),
	m_funUpdateStatusBodyTracker(funUpdateStatusBodyTracker),
	m_funProcessBody(funProcessBody),
	m_funProcessIMU(funProcessIMU)
{
	setParams();
}


KinectAzure::~KinectAzure()
{
	ReleaseDefaultSensor();
}

void KinectAzure::Terminate()
{
	m_bTerminating = true;
	m_ThreadSkeleton.join();
	m_ThreadImu.join();
	ReleaseDefaultSensor();
	
}

void KinectAzure::SkeletonProc()
{
	while (!m_bTerminating) 
	{
		EnsureDefaultSensor();
		if (m_Kinect && m_KinectBodyTracker)
			SkeletonUpdate();
		else
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	
}

void KinectAzure::ImuProc()
{
	while (!m_bTerminating)
	{
		if (m_Kinect)
			ImuUpdate();
		else
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
}


void KinectAzure::setParams()
{

	// Configure Kinect device
	m_KinectConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;

	int depth_mode_selection = K4A_DEPTH_MODE_NFOV_UNBINNED;
	Config::Instance()->assign("k4a/depth_mode", depth_mode_selection);
	m_KinectConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
	switch (depth_mode_selection)
	{
	case 1:
		m_KinectConfig.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED; // 320x288
		break;
	case 2:
		m_KinectConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED; // 640x576
		break;
	case 3:
		m_KinectConfig.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED; // 512x512
		break;
	case 4:
		m_KinectConfig.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED; // 1024x1024
		break;
	default:
		m_KinectConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED; // 640x576

	}
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
void KinectAzure::EnsureDefaultSensor()
{
	k4a_result_t result;
	if (!m_Kinect)
	{
		// Open Kinect device
		result = k4a_device_open(K4A_DEVICE_DEFAULT, &m_Kinect);
		if (K4A_FAILED(result))
		{
			if (m_funUpdateStatusKinect) m_funUpdateStatusKinect(L"Failed to open k4a device.");
			return;
		}

		// Start the camera
		result = k4a_device_start_cameras(m_Kinect, &m_KinectConfig);
		if (K4A_FAILED(result))
		{
			if (m_funUpdateStatusKinect) m_funUpdateStatusKinect(L"k4a device was open, but failed to start cameras.");
			k4a_device_close(m_Kinect);
			m_Kinect = NULL;
			return;
		}
		
		result = k4a_device_start_imu(m_Kinect);
		if (K4A_FAILED(result))
		{
			if (m_funUpdateStatusKinect) m_funUpdateStatusKinect(L"k4a device was open, but failed to start imus.");
			k4a_device_close(m_Kinect);
			m_Kinect = NULL;
			return;
		}

		// Obtain calibration data
		k4a_device_get_calibration(m_Kinect, m_KinectConfig.depth_mode, m_KinectConfig.color_resolution, &m_KinectCalibration);
		if (m_funUpdateStatusKinect) m_funUpdateStatusKinect(L"k4a device is open.");
	}

	// Create body tracker
	if (!m_KinectBodyTracker)
	{
		if (m_funUpdateStatusBodyTracker) m_funUpdateStatusBodyTracker(L"Creating body tracker.");
		result = k4abt_tracker_create(&m_KinectCalibration, &m_KinectBodyTracker);
		if (K4A_FAILED(result))
		{
			if (m_funUpdateStatusBodyTracker) m_funUpdateStatusBodyTracker(L"Failed to create body tracker.");
			m_KinectBodyTracker = NULL;
			return;
		}
		else
			if (m_funUpdateStatusBodyTracker) m_funUpdateStatusBodyTracker(L"Successfully created body tracker.");
	}
}

void KinectAzure::ReleaseDefaultSensor()
{

	if (m_KinectBodyTracker)
	{
		//k4abt_tracker_destroy(m_KinectBodyTracker);
		m_KinectBodyTracker = NULL;
	}
	
	if (m_Kinect)
	{
		k4a_device_stop_imu(m_Kinect);
		k4a_device_stop_cameras(m_Kinect);
		k4a_device_close(m_Kinect);
		m_Kinect = NULL;
	}
}

void KinectAzure::SkeletonUpdate()
{
	int32_t timeout_ms = 1000;

	// Read a sensor capture  
	k4a_capture_t capture;
	k4a_wait_result_t capture_result = k4a_device_get_capture(m_Kinect, &capture, timeout_ms);

	if (capture_result == K4A_WAIT_RESULT_SUCCEEDED)
	{
		k4a_wait_result_t queue_result = k4abt_tracker_enqueue_capture(m_KinectBodyTracker, capture, timeout_ms);
		k4a_capture_release(capture);
		
		if (queue_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			clock_t execution_time = clock();

			k4abt_frame_t body_frame = NULL;
			k4a_wait_result_t pop_result = k4abt_tracker_pop_result(m_KinectBodyTracker, &body_frame, timeout_ms);
			
			execution_time = clock() - execution_time;
			static std::array<clock_t, 5> arr_time;
			static size_t i;
			arr_time[i++] = execution_time;
			if (i == arr_time.size())
			{
				std::wstring wstr; 
				std::for_each(arr_time.cbegin(), arr_time.cend(), 
					[&wstr](clock_t t) {wstr += (wstr.empty() ? L"": L", ") + std::to_wstring(t); });
				wstr = L"Execution time for pop: " + wstr + L" ms";
				if (m_funUpdateStatusBodyTracker) m_funUpdateStatusBodyTracker(wstr.c_str());
				i = 0;
			}
			
			if (pop_result == K4A_WAIT_RESULT_SUCCEEDED)
			{
				uint64_t timestamp_usec = k4abt_frame_get_timestamp_usec(body_frame);
				const size_t MAX_NUM_BODIES = 6;
				size_t num_bodies = min(MAX_NUM_BODIES, k4abt_frame_get_num_bodies(body_frame));
				uint32_t body_ids[MAX_NUM_BODIES] = {};
				k4abt_skeleton_t skeletons[MAX_NUM_BODIES] = {};

				k4a_result_t skeleton_result = K4A_RESULT_SUCCEEDED;
				for (size_t i = 0; i < num_bodies; i++)
				{
					body_ids[i] = k4abt_frame_get_body_id(body_frame, i);
					k4a_result_t result = k4abt_frame_get_body_skeleton(body_frame, i, &skeletons[i]);
					if (K4A_FAILED(result)) {
						skeleton_result = K4A_RESULT_FAILED;
						break;
					}
				}
				if (K4A_SUCCEEDED(skeleton_result) && m_funProcessBody)
					m_funProcessBody(timestamp_usec, num_bodies, skeletons, body_ids);

				k4abt_frame_release(body_frame);
			}
		}


	}
	else if (capture_result == K4A_WAIT_RESULT_TIMEOUT)
	{
		// Presumably disconnected
		k4a_device_stop_cameras(m_Kinect);
		k4a_device_close(m_Kinect);
		m_Kinect = NULL;
	}
}

void KinectAzure::ImuUpdate()
{
	
	int32_t timeout_ms = 1000;
	k4a_imu_sample_t imu_data;
	k4a_wait_result_t imu_result = k4a_device_get_imu_sample(m_Kinect, &imu_data, timeout_ms);
	if (imu_result == K4A_WAIT_RESULT_SUCCEEDED)
	{
		m_funProcessIMU(imu_data);
	}
}

const k4a_calibration_t * KinectAzure::GetKinectCalibrationPointer()
{
	return &m_KinectCalibration;
}


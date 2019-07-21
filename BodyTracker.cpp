//------------------------------------------------------------------------------
// <copyright file="BodyBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <string>
#include <Windows.h>
#include "RosSocket.h"
#include "BodyTracker.h"
#include "Config.h"

static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(    
	_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd
)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

	try {
		BodyTracker application;
		application.Run(hInstance, nShowCmd);
	}
	catch (std::runtime_error & error) {
		ErrorExit((LPTSTR)std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(error.what()).c_str());
	}
	

}

/// <summary>
/// Constructor
/// </summary>
BodyTracker::BodyTracker() :
	m_hWnd(NULL),
	m_nStartTime(GetTickCount64()),
	m_nLastCounter(0),
	m_nFramesSinceUpdate(0),
	m_fFreq(0),
	m_nNextStatusTime(0LL),
	m_KinectAzure(
		std::bind(&BodyTracker::PrintMessage, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&BodyTracker::ProcessBody, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
		std::bind(&BodyTracker::ProcessIMU, this, std::placeholders::_1),
		std::bind(&BodyTracker::BroadcastStaticTf, this)
	),
	m_pD2DFactory(NULL),
	m_pRenderTarget(NULL),
	m_pBrushJointTracked(NULL),
	m_pBrushBoneTracked(NULL),
	m_pSyncSocket(nullptr),
	m_pRosSocket(nullptr)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }

	m_pSyncSocket = new SyncSocket();

	setParams();

	for (int i = 0; i < SCT_Count; i++)
		m_hWndStaticControls[i] = NULL;

	
}
  

/// <summary>
/// Destructor
/// </summary>
BodyTracker::~BodyTracker()
{
	delete m_pSyncSocket;
	delete m_pRosSocket;
	m_pRosSocket = nullptr;

    DiscardDirect2DResources();

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);
	m_hWnd = NULL;
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int BodyTracker::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"BodyTrackerAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)BodyTracker::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

	RECT rc;
	GetWindowRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rc);
	int heightButton = 40, widthButton = 180;
	const int xButtonLeft = rc.left + 20;
	const int yButtonTop = rc.bottom + 40;
	const int yButtonSep = 20, xButtonSep = 30;
	const int yButtonBottom = yButtonTop + (heightButton + yButtonSep) * 1.5;

	struct ControlProperty {
		HWND * phWnd;
		LPCWSTR className;
		LPCWSTR text;
		DWORD dwStyle;
		int w; 
		int h;
	};
	
	ControlProperty BCPs[] = { 
		{ &m_hWndButtonFollow, L"BUTTON", L"Start following",
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, widthButton, heightButton },

		{ &m_hWndButtonManual, L"BUTTON", L"Start Manual",
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, widthButton, heightButton },

		{ &m_hWndButtonReserved, L"BUTTON", L"Reserved",
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, widthButton, heightButton },

		{ &m_hWndButtonCalibrate, L"BUTTON", L"Start Calibration",
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, widthButton, heightButton },

		{ &m_hWndButtonOpenConfig, L"BUTTON", L"Open Config", 
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, widthButton, heightButton },

		{ &m_hWndButtonLoad, L"BUTTON", L"Load Config", 
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, widthButton, heightButton },

		{ &m_hWndButtonTestCalibSolver, L"BUTTON", L"Test CalibSolver",
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, widthButton, heightButton },

		{ &m_hWndButtonExit, L"BUTTON", L"Exit",
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON, widthButton, heightButton }
	};

	for (int i = 0, x = xButtonLeft, y = yButtonTop;
		i < sizeof(BCPs) / sizeof(BCPs[0]); 
		y += BCPs[i].h + yButtonSep, i++)
	{
		if (y > yButtonBottom) {
			y = yButtonTop;
			x += BCPs[i].w + xButtonSep;
		}
		*(BCPs[i].phWnd) = CreateWindow(
			BCPs[i].className,  // Predefined class; Unicode assumed 
			BCPs[i].text,      // Button text 
			BCPs[i].dwStyle,  // Styles 
			x,			// x position 
			y,			// y position 
			BCPs[i].w,		// Button width
			BCPs[i].h,		// Button height
			hWndApp,    // Parent window
			NULL,       // No menu.
			(HINSTANCE)GetWindowLongPtr(hWndApp, GWLP_HINSTANCE),
			NULL);      // Pointer not needed.;
	}

	// List of static controls
	const int widthStatic = 700;
	const int heightStatic = 20;
	const int xStaticLeft = rc.right + 100;
	const int yStaticTop = rc.top;
	for (int i = 0, x = xStaticLeft, y = yStaticTop;
		i < SCT_Count;
		y += heightStatic + yButtonSep, i++)
	{
		m_hWndStaticControls[i] = CreateWindow(
			L"STATIC",  // Predefined class; Unicode assumed 
			L"",				// Static text 
			WS_VISIBLE | WS_CHILD,  // Styles 
			x,			// x position 
			y,			// y position 
			widthStatic,		// Static width
			heightStatic,		// Static height
			hWndApp,    // Parent window
			NULL,       // No menu.
			(HINSTANCE)GetWindowLongPtr(hWndApp, GWLP_HINSTANCE),
			NULL);      // Pointer not needed.;
	}


	// Register hotkeys
	//RegisterHotKey(hWndApp, 1, MOD_CONTROL | MOD_NOREPEAT, 'F');
	//RegisterHotKey(hWndApp, 2, MOD_CONTROL | MOD_NOREPEAT, 'C');
	//RegisterHotKey(hWndApp, 3, MOD_NOREPEAT, VK_F5);

    // Show window
    ShowWindow(hWndApp, SW_MAXIMIZE);//nCmdShow


	// Init Direct2D
	D2D1CreateFactory(D2D1_FACTORY_TYPE_MULTI_THREADED, &m_pD2DFactory);

	// Get and initialize the default Kinect sensor
	//EnsureDefaultSensor();

	//
	m_pSyncSocket->init(hWndApp);

	// Initialize RosSocket
	EnsureRosSocket();
	
    // Main message loop
    while (WM_QUIT != msg.message)
    {
		// Odroid Timestamp
		if (m_pSyncSocket)
			m_pSyncSocket->receive();

		Update();

		//calibrate();
		Sleep(2);
        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
			if (WM_QUIT == msg.message) break;
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
				continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

void BodyTracker::setParams()
{
	m_KinectAzure.setParams();
}

void BodyTracker::onPressingButtonFollow()
{

}

void BodyTracker::onPressingButtonCalibrate()
{

}

void BodyTracker::onPressingButtonManual()
{
	
}

void BodyTracker::updateButtons()
{

}

/// <summary>
/// Main processing function
/// </summary>
void BodyTracker::Update()
{
	static INT64 timePrev = GetTickCount64();
	if (GetTickCount64() - timePrev > 500)
	{
		EnsureRosSocket();
		timePrev = GetTickCount64();
	}

}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK BodyTracker::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    BodyTracker* pThis = NULL;

    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<BodyTracker*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<BodyTracker*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }
	
    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK BodyTracker::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

			SetFocus(hWnd);
        }
        break;

        case WM_DESTROY:
			//m_bTerminating = true;
			m_hWnd = NULL;
			m_KinectAzure.Terminate();
            // Quit the main message pump
            PostQuitMessage(0);
            break;

		case WM_HOTKEY:
		{
			switch (wParam)
			{
			case 1:
				onPressingButtonFollow();
				break;
			case 2:
				onPressingButtonManual();
				//onPressingButtonCalibrate();
				break;
			case 3:
				DestroyWindow(hWnd);
			}
			break;
		}
		case WM_COMMAND:
			switch (wParam)
			{
			case BN_CLICKED: //WM_COMMAND
				HWND hButton = (HWND)lParam;
				if (m_hWndButtonFollow == hButton)
				{
					onPressingButtonFollow();
				}
				else if (m_hWndButtonCalibrate == hButton)
				{
					onPressingButtonCalibrate();
				}
				else if (m_hWndButtonManual == hButton)
				{
					onPressingButtonManual();
				}
				else if (m_hWndButtonOpenConfig == hButton)
				{
					// Open Config Button
					system("notepad.exe config.txt");
				}
				else if (m_hWndButtonTestCalibSolver == hButton) {
					
				}
				else if (m_hWndButtonLoad == hButton)
				{
					// Load button
					Config::Instance()->load();
					Config::resetCounter();
					setParams();
					int cnt = Config::getUpdateCount();
					TCHAR pszText[32];
					StringCchPrintf(pszText, 32, L"%d parameter%s updated.", cnt, cnt > 1 ? L"s" : L"");
					PrintMessage(SCT_Params, pszText);
				}
				else if (m_hWndButtonExit == hButton)
				{
					// Exit button
					DestroyWindow(hWnd);
				}
				break;
			}
			break;
		default:
			break;
    }

    return FALSE;
}

void BodyTracker::EnsureRosSocket()
{
	
	if (m_pRosSocket && m_pRosSocket->getStatus() == RSS_Failed)
	{
		delete m_pRosSocket;
		m_pRosSocket = nullptr;
	}

	if (!m_pRosSocket) 
	{
		bool bRosSocketEnabled;
		Config::Instance()->assign("RosSocket/enabled", bRosSocketEnabled);
		if (bRosSocketEnabled)
		{
			m_pRosSocket = new RosSocket();
			m_pRosSocket->setStatusUpdatingFun(std::bind(&BodyTracker::PrintMessage, this, std::placeholders::_1, std::placeholders::_2));
		}
		else
		{
			PrintMessage(SCT_RosSocket, L"RosSocket is disabled");
		}
	}
}



/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame in usec</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// </summary>
void BodyTracker::ProcessBody(uint64_t nTime, int nBodyCount, const k4abt_skeleton_t *pSkeleton, const uint32_t * pID)
{
	int iClosest = -1; // the index of the body closest to the camera
	float dSqrMin = 25.0; // squared x-z-distance of the closest body
	INT64 t0Windows = GetTickCount64();
	if (m_pRosSocket && m_pRosSocket->getStatus() == RSS_Connected)
	{
		// Publish only the body with min dist
		float min_dist = std::numeric_limits<float>::infinity();
		int min_dist_i = 0;
		for (int i = 0; i < nBodyCount; i++) {
			const k4a_float3_t & position_pelvis = pSkeleton[i].joints[K4ABT_JOINT_PELVIS].position;
			float dist = hypot(position_pelvis.xyz.x, position_pelvis.xyz.z);
			if (min_dist > dist)
			{
				min_dist = dist;
				min_dist_i = i;
			}
		}
		m_pRosSocket->publishMsgSkeleton(pSkeleton[min_dist_i], pID[min_dist_i], nTime);
	}

    if (m_hWnd)
    {
        HRESULT hr = EnsureDirect2DResources();

        if (SUCCEEDED(hr) && m_pRenderTarget)
        {
            m_pRenderTarget->BeginDraw();
            m_pRenderTarget->Clear();

            RECT rct;
            GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
            int width = rct.right;
            int height = rct.bottom;

            for (int i = 0; i < nBodyCount; ++i)
            {
                k4abt_skeleton_t const &skeleton = pSkeleton[i];
                D2D1_POINT_2F jointPoints[K4ABT_JOINT_COUNT];

                for (int j = 0; j < K4ABT_JOINT_COUNT; ++j)
                {
                    jointPoints[j] = BodyToScreen(skeleton.joints[j].position, width, height);
                }

                DrawBody(jointPoints);

				// Find the closest body, if any.
				float px = skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.x;
				float pz = skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.z;
				float dSqr = px * px + pz * pz;
				if (dSqrMin > dSqr)
				{
					dSqrMin = dSqr;
					iClosest = i;
				}
            }

            hr = m_pRenderTarget->EndDraw();

            // Device lost, need to recreate the render target
            // We'll dispose it now and retry drawing
            if (D2DERR_RECREATE_TARGET == hr)
            {
                hr = S_OK;
                DiscardDirect2DResources();
            }
        }
		
        double fps = 0.0;

        LARGE_INTEGER qpcNow = {0};
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow))
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

		
		WCHAR szStatusMessage[64];
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage),
			L" FPS = %0.2f; Time = %.0f s; Sync = %d", 
			fps, (GetTickCount64() - m_nStartTime) / 1.0e3, m_pSyncSocket->m_nPacketCount);

		if (SetStatusMessage(szStatusMessage, 500, false))
		{
			m_nLastCounter = qpcNow.QuadPart;
			m_nFramesSinceUpdate = 0;
		}
		
		
    }

	
}

void BodyTracker::ProcessIMU(const k4a_imu_sample_t & ImuSample)
{
	if (m_pRosSocket && m_pRosSocket->getStatus() == RSS_Connected)
	{
		m_pRosSocket->publishMsgImu(ImuSample);
	}

	double fps = 0.0;
	LARGE_INTEGER qpcNow = { 0 };
	static INT64 nNextStatusTime;
	static DWORD nSamplesSinceUpdate;
	static INT64 nLastCounter;
	if (m_fFreq)
	{
		if (QueryPerformanceCounter(&qpcNow))
		{
			if (nLastCounter)
			{
				nSamplesSinceUpdate++;
				fps = m_fFreq * nSamplesSinceUpdate / double(qpcNow.QuadPart - nLastCounter);
			}
		}
	}
	INT64 now = GetTickCount64();
	if (nNextStatusTime <= now)
	{
		PrintMessage(SCT_IMU, (L"IMU data being sampled at " + std::to_wstring(fps) + L" Hz").c_str());
		nNextStatusTime = now + 500;
		nLastCounter = qpcNow.QuadPart;
		nSamplesSinceUpdate = 0;
	}
}

void BodyTracker::BroadcastStaticTf()
{
	const k4a_calibration_t * pCalibration = m_KinectAzure.GetKinectCalibrationPointer();
	if (m_pRosSocket && m_pRosSocket->getStatus() == RSS_Connected && pCalibration)
	{
		m_pRosSocket->broadcastDepthTf(pCalibration);
		m_pRosSocket->broadcastImuTf(pCalibration);
	}
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool BodyTracker::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS1, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }
    return false;
}

void BodyTracker::PrintMessage(static_control_type SCT, const wchar_t * szMessage)
{
	DWORD dw;
	if (m_hWnd)
	{
		const size_t BUFFER_LEN = 128;
		wchar_t pszText[BUFFER_LEN];
		StringCchPrintf(pszText, BUFFER_LEN, L"[%.3fs]: %s",
			(GetTickCount64() - m_nStartTime) / 1.0e3,
			szMessage);
		SetWindowText(m_hWndStaticControls[SCT], pszText);
	}
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>S_OK if successful, otherwise an error code</returns>
HRESULT BodyTracker::EnsureDirect2DResources()
{
    HRESULT hr = S_OK;

    if (m_pD2DFactory && !m_pRenderTarget)
    {
        RECT rc;
        GetWindowRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rc);  

        int width = rc.right - rc.left;
        int height = rc.bottom - rc.top;
        D2D1_SIZE_U size = D2D1::SizeU(width, height);
        D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
        rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

        // Create a Hwnd render target, in order to render to the window set in initialize
        hr = m_pD2DFactory->CreateHwndRenderTarget(
            rtProps,
            D2D1::HwndRenderTargetProperties(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), size),
            &m_pRenderTarget
        );

        if (FAILED(hr))
        {
            SetStatusMessage(L"Couldn't create Direct2D render target!", 10000, true);
            return hr;
        }

        // light green
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
    }

    return hr;
}

/// <summary>
/// Dispose Direct2d resources 
/// </summary>
void BodyTracker::DiscardDirect2DResources()
{
    SafeRelease(m_pRenderTarget);

    SafeRelease(m_pBrushJointTracked);
    SafeRelease(m_pBrushBoneTracked);
}

/// <summary>
/// Converts a body point to screen space
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
D2D1_POINT_2F BodyTracker::BodyToScreen(const k4a_float3_t& point3d, int width_rendering, int height_rendering)
{
	k4a_float2_t point2d;
	int valid;
	const k4a_calibration_t * pKinectCalibration = m_KinectAzure.GetKinectCalibrationPointer();
	k4a_calibration_3d_to_2d(pKinectCalibration, &point3d, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &point2d, &valid);
	float width_actual = pKinectCalibration->depth_camera_calibration.resolution_width;
	float height_actual = pKinectCalibration->depth_camera_calibration.resolution_height;
	float ratio_width = static_cast<float>(width_rendering) / width_actual;
	float ratio_height = static_cast<float>(height_rendering) / height_actual;
	float ratio_min = min(ratio_width, ratio_height);

	float px = width_rendering / 2 + (-point2d.xy.x + width_actual / 2) * ratio_min;
	float py = point2d.xy.y * ratio_min;
	return D2D1::Point2F(px, py);
}

/// <summary>
/// Draws a body 
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
void BodyTracker::DrawBody(const D2D1_POINT_2F* pJointPoints)
{
    // Draw the bones

	// Head
	DrawBone(pJointPoints, K4ABT_JOINT_HEAD, K4ABT_JOINT_NOSE);
	DrawBone(pJointPoints, K4ABT_JOINT_HEAD, K4ABT_JOINT_EYE_LEFT);
	DrawBone(pJointPoints, K4ABT_JOINT_HEAD, K4ABT_JOINT_EYE_RIGHT);
	DrawBone(pJointPoints, K4ABT_JOINT_HEAD, K4ABT_JOINT_EYE_LEFT);
	DrawBone(pJointPoints, K4ABT_JOINT_HEAD, K4ABT_JOINT_EYE_LEFT);
    // Torso
    DrawBone(pJointPoints, K4ABT_JOINT_HEAD, K4ABT_JOINT_NECK);
    DrawBone(pJointPoints, K4ABT_JOINT_NECK, K4ABT_JOINT_SPINE_CHEST);
    DrawBone(pJointPoints, K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_SPINE_NAVAL);
    DrawBone(pJointPoints, K4ABT_JOINT_SPINE_NAVAL, K4ABT_JOINT_PELVIS);
    DrawBone(pJointPoints, K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_CLAVICLE_RIGHT);
    DrawBone(pJointPoints, K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_CLAVICLE_LEFT);
    DrawBone(pJointPoints, K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_RIGHT);
    DrawBone(pJointPoints, K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_LEFT);
    
    // Right Arm    
	DrawBone(pJointPoints, K4ABT_JOINT_CLAVICLE_RIGHT, K4ABT_JOINT_SHOULDER_RIGHT);
	DrawBone(pJointPoints, K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_ELBOW_RIGHT);
    DrawBone(pJointPoints, K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_WRIST_RIGHT);
    // Left Arm
	DrawBone(pJointPoints, K4ABT_JOINT_CLAVICLE_LEFT, K4ABT_JOINT_SHOULDER_LEFT);
	DrawBone(pJointPoints, K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_ELBOW_LEFT);
	DrawBone(pJointPoints, K4ABT_JOINT_ELBOW_LEFT, K4ABT_JOINT_WRIST_LEFT);

    // Right Leg
    DrawBone(pJointPoints, K4ABT_JOINT_HIP_RIGHT, K4ABT_JOINT_KNEE_RIGHT);
    DrawBone(pJointPoints, K4ABT_JOINT_KNEE_RIGHT, K4ABT_JOINT_ANKLE_RIGHT);
    DrawBone(pJointPoints, K4ABT_JOINT_ANKLE_RIGHT, K4ABT_JOINT_FOOT_RIGHT);

    // Left Leg
    DrawBone(pJointPoints, K4ABT_JOINT_HIP_LEFT, K4ABT_JOINT_KNEE_LEFT);
    DrawBone(pJointPoints, K4ABT_JOINT_KNEE_LEFT, K4ABT_JOINT_ANKLE_LEFT);
    DrawBone(pJointPoints, K4ABT_JOINT_ANKLE_LEFT, K4ABT_JOINT_FOOT_LEFT);

    // Draw the joints
    for (int i = 0; i < K4ABT_JOINT_COUNT; ++i)
    {
        D2D1_ELLIPSE ellipse = D2D1::Ellipse(pJointPoints[i], c_JointThickness, c_JointThickness);
        m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
    }
}

/// <summary>
/// Draws one bone of a body (joint to joint)
/// </summary>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="joint0">one joint of the bone to draw</param>
/// <param name="joint1">other joint of the bone to draw</param>
void BodyTracker::DrawBone(const D2D1_POINT_2F* pJointPoints, k4abt_joint_id_t joint0, k4abt_joint_id_t joint1)
{
	m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness);
}

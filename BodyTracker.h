// Renamed to BodyTracker.h

//------------------------------------------------------------------------------
// <copyright file="BodyBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "SyncSocket.h"
#include "Config.h"
#include "SyncSocket.h"
#include <array>
#include "KinectAzure.h"


void ErrorExit(LPTSTR lpszFunction)
{
	// Retrieve the system error message for the last-error code

	LPVOID lpMsgBuf;
	LPVOID lpDisplayBuf;
	DWORD dw = GetLastError();

	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER |
		FORMAT_MESSAGE_FROM_SYSTEM |
		FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		dw,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPTSTR)&lpMsgBuf,
		0, NULL);

	// Display the error message and exit the process

	lpDisplayBuf = (LPVOID)LocalAlloc(LMEM_ZEROINIT,
		(lstrlen((LPCTSTR)lpMsgBuf) + lstrlen((LPCTSTR)lpszFunction) + 40) * sizeof(TCHAR));
	StringCchPrintf((LPTSTR)lpDisplayBuf,
		LocalSize(lpDisplayBuf) / sizeof(TCHAR),
		TEXT("%s failed with error %d: %s"),
		lpszFunction, dw, lpMsgBuf);
	MessageBox(NULL, (LPCTSTR)lpDisplayBuf, TEXT("Error"), MB_OK);

	LocalFree(lpMsgBuf);
	LocalFree(lpDisplayBuf);
	ExitProcess(dw);
}


class BodyTracker
{
    static const int        cDepthWidth  = 512;
    static const int        cDepthHeight = 424;

public:
    BodyTracker();
    ~BodyTracker();
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
    int                     Run(HINSTANCE hInstance, int nCmdShow);

private:
    HWND                    m_hWnd;
    INT64                   m_nStartTime;
    INT64                   m_nLastCounter;
    double                  m_fFreq;
    INT64                   m_nNextStatusTime;
    DWORD                   m_nFramesSinceUpdate;

    // KinectAzure
	KinectAzure             m_KinectAzure;

    // Direct2D
    ID2D1Factory*           m_pD2DFactory;

    // Body/hand drawing
    ID2D1HwndRenderTarget*  m_pRenderTarget;
    ID2D1SolidColorBrush*   m_pBrushJointTracked;
    ID2D1SolidColorBrush*   m_pBrushBoneTracked;

	//
	SyncSocket*				m_pSyncSocket;

	// Interface
	HWND					m_hWndButtonFollow;
	HWND					m_hWndButtonManual;
	HWND					m_hWndButtonReserved;
	HWND					m_hWndButtonCalibrate;
	HWND					m_hWndButtonOpenConfig;
	HWND					m_hWndButtonLoad;
	HWND					m_hWndButtonTestCalibSolver;
	HWND					m_hWndButtonExit;

	HWND                    m_hWndStaticControls[SCT_Count];

	// ROS Socket
	RosSocket*				m_pRosSocket;

	void                    setParams();

	inline void				onPressingButtonFollow();
	inline void				onPressingButtonCalibrate();
	inline void				onPressingButtonManual();
	inline void				updateButtons();
    /// <summary>
    /// Main processing function
    /// </summary>
    void                    Update();

	void EnsureRosSocket();

    
    /// <summary>
    /// Handle new body data
	/// <param name="nTime">timestamp of frame in msec</param>
    /// <param name="nBodyCount">body data count</param>
    /// <param name="ppBodies">body data in frame</param>
    /// </summary>
    void ProcessBody(uint64_t nTime, int nBodyCount, const k4abt_skeleton_t *pSkeleton, const uint32_t * pID);
    void ProcessIMU(const k4a_imu_sample_t & ImuSample);

    /// <summary>
    /// Set the status bar message
    /// </summary>
    /// <param name="szMessage">message to display</param>
    /// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
    /// <param name="bForce">force status update</param>
    bool SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);

	// Print to the message textbox
	void PrintMessage(static_control_type SCT, const wchar_t * szMessage);

    /// <summary>
    /// Ensure necessary Direct2d resources are created
    /// </summary>
    /// <returns>S_OK if successful, otherwise an error code</returns>
    HRESULT EnsureDirect2DResources();

    /// <summary>
    /// Dispose Direct2d resources 
    /// </summary>
    void DiscardDirect2DResources();

    /// <summary>
    /// Converts a body point to screen space
    /// </summary>
    /// <param name="bodyPoint">body point to tranform</param>
    /// <param name="width">width (in pixels) of output buffer</param>
    /// <param name="height">height (in pixels) of output buffer</param>
    /// <returns>point in screen-space</returns>
    D2D1_POINT_2F           BodyToScreen(const k4a_float3_t& bodyPoint, int width, int height);

    /// <summary>
    /// Draws a body 
    /// </summary>
    /// <param name="pJointPoints">joint positions converted to screen space</param>
    void DrawBody(const D2D1_POINT_2F* pJointPoints);


    /// <summary>
    /// Draws one bone of a body (joint to joint)
    /// </summary>
    /// <param name="pJointPoints">joint positions converted to screen space</param>
    /// <param name="joint0">one joint of the bone to draw</param>
    /// <param name="joint1">other joint of the bone to draw</param>
    void DrawBone(const D2D1_POINT_2F* pJointPoints, k4abt_joint_id_t joint0, k4abt_joint_id_t joint1);

};

// Copyright (c) 2013-2018 United States Government as represented by the Administrator of the
// National Aeronautics and Space Administration. All Rights Reserved.
//
// DISCLAIMERS
//     No Warranty: THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY KIND,
//     EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, ANY WARRANTY THAT
//     THE SUBJECT SOFTWARE WILL CONFORM TO SPECIFICATIONS, ANY IMPLIED WARRANTIES OF
//     MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY
//     THAT THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT DOCUMENTATION, IF PROVIDED,
//     WILL CONFORM TO THE SUBJECT SOFTWARE. THIS AGREEMENT DOES NOT, IN ANY MANNER, CONSTITUTE AN
//     ENDORSEMENT BY GOVERNMENT AGENCY OR ANY PRIOR RECIPIENT OF ANY RESULTS, RESULTING DESIGNS,
//     HARDWARE, SOFTWARE PRODUCTS OR ANY OTHER APPLICATIONS RESULTING FROM USE OF THE SUBJECT
//     SOFTWARE.  FURTHER, GOVERNMENT AGENCY DISCLAIMS ALL WARRANTIES AND LIABILITIES REGARDING
//     THIRD-PARTY SOFTWARE, IF PRESENT IN THE ORIGINAL SOFTWARE, AND DISTRIBUTES IT "AS IS."
//
//     Waiver and Indemnity: RECIPIENT AGREES TO WAIVE ANY AND ALL CLAIMS AGAINST THE UNITED STATES
//     GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY PRIOR RECIPIENT.  IF
//     RECIPIENT'S USE OF THE SUBJECT SOFTWARE RESULTS IN ANY LIABILITIES, DEMANDS, DAMAGES, EXPENSES
//     OR LOSSES ARISING FROM SUCH USE, INCLUDING ANY DAMAGES FROM PRODUCTS BASED ON, OR RESULTING
//     FROM, RECIPIENT'S USE OF THE SUBJECT SOFTWARE, RECIPIENT SHALL INDEMNIFY AND HOLD HARMLESS THE
//     UNITED STATES GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY PRIOR RECIPIENT,
//     TO THE EXTENT PERMITTED BY LAW.  RECIPIENT'S SOLE REMEDY FOR ANY SUCH MATTER SHALL BE THE
//     IMMEDIATE, UNILATERAL TERMINATION OF THIS AGREEMENT.
//
// X-Plane API
// Copyright(c) 2008, Sandy Barbour and Ben Supnik All rights reserved.
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files(the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and / or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Neither the names of the authors nor that of X - Plane or Laminar Research
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission from the authors or
//     Laminar Research, respectively.


// X-Plane Connect Plugin
//
// DESCRIPTION
//     XPCPlugin Facilitates Communication to and from the XPlane
//
// INSTRUCTIONS
//     See Readme.md in the root of this repository or the wiki hosted on GitHub at
//     https://github.com/nasa/XPlaneConnect/wiki for requirements, installation instructions,
//     and detailed documentation.
//
// CONTACT
//     For questions email Christopher Teubert (christopher.a.teubert@nasa.gov)
//
// CONTRIBUTORS
//     CT: Christopher Teubert (christopher.a.teubert@nasa.gov)
//     JW: Jason Watkins (jason.w.watkins@nasa.gov)

// XPC Includes
#include "DataManager.h"
#include "Drawing.h"
#include "Log.h"
#include "MessageHandlers.h"
#include "UDPSocket.h"
#include "Timer.h"

// XPLM Includes
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

// System Includes
#include <cstdlib>
#include <cstring>
#include <cmath>
#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

#define RECVPORT 49009 // Port that the plugin receives commands on
#define OPS_PER_CYCLE 20 // Max Number of operations per cycle

#define XPC_PLUGIN_VERSION "1.4.3"


using namespace std;

XPC::UDPSocket* sock = NULL;
XPC::Timer* timer = NULL;

double start;
double lap;
static double timeConvert = 0.0;
int benchmarkingSwitch = 0; // 1 = time for operations, 2 = time for op + cycle;

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc);
PLUGIN_API void	XPluginStop(void);
PLUGIN_API void XPluginDisable(void);
PLUGIN_API int XPluginEnable(void);
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho, int inMessage, void* inParam);

static float XPCFlightLoopCallbackBeforePhysics(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);
static float XPCFlightLoopCallbackAfterPhysics(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);
XPLMFlightLoopID SDK210_XPLMTempFlightLoopID = NULL;
XPLMFlightLoopID SDK210_XPLMTempFlightLoopAfterPhysicsID = NULL;

// Needed to backup POSI and other messages and write them again after physics (or if no POSI msg was received in a frame)
bool writePosiAfterPhysics = false;
XPC::Message msgPOSI;
XPC::Message msgVX;
XPC::Message msgVY;
XPC::Message msgVZ;
XPC::Message msgAX;
XPC::Message msgAY;
XPC::Message msgAZ;
XPC::Message msgP;
XPC::Message msgQ;
XPC::Message msgR;


PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc)
{
	strcpy(outName, "Lilium to X-Plane Connect [v1.4.3]");
	strcpy(outSig, "NASA.XPlaneConnect");
	strcpy(outDesc, "X Plane Communications Toolbox\nCopyright (c) 2013-2018 United States Government as represented by the Administrator of the National Aeronautics and Space Administration. All Rights Reserved.");

#if (__APPLE__)
	if ( timeConvert <= 1e-9 ) // is about 0
	{
		mach_timebase_info_data_t timeBase;
		(void)mach_timebase_info(&timeBase);
		timeConvert = (double)timeBase.numer /
		(double)timeBase.denom /
		1000000000.0;
	}
#endif
	XPC::Log::Initialize(XPC_PLUGIN_VERSION);
	XPC::Log::WriteLine(LOG_INFO, "EXEC", "Plugin Start");
	XPC::DataManager::Initialize();

	return 1;
}

PLUGIN_API void	XPluginStop(void)
{
	XPC::Log::WriteLine(LOG_INFO, "EXEC", "Plugin Shutdown");
	XPC::Log::Close();
}

PLUGIN_API void XPluginDisable(void)
{
	XPLMDestroyFlightLoop(SDK210_XPLMTempFlightLoopID);
	XPLMDestroyFlightLoop(SDK210_XPLMTempFlightLoopAfterPhysicsID);

	// Close sockets
	delete sock;
	sock = NULL;

	// Stop rendering messages to screen.
	XPC::Drawing::ClearMessage();

	// Stop rendering waypoints to screen.
	XPC::Drawing::ClearWaypoints();

	XPC::Log::WriteLine(LOG_INFO, "EXEC", "Plugin Disabled, sockets closed");
	
	timer->stop();
	delete timer;
	timer = NULL;
}

PLUGIN_API int XPluginEnable(void)
{
	// Open sockets
	sock = new XPC::UDPSocket(RECVPORT);
	timer = new XPC::Timer();

	XPC::MessageHandlers::SetSocket(sock);

	XPC::Log::WriteLine(LOG_INFO, "EXEC", "Plugin Enabled, sockets opened");
	if (benchmarkingSwitch > 0)
	{
		XPC::Log::FormatLine(LOG_INFO, "EXEC", "Benchmarking Enabled (Verbosity: %i)", benchmarkingSwitch);
	}
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Debug Logging Enabled (Verbosity: %i)", LOG_LEVEL);

	float interval = -1; // Call every frame
	void* refcon = NULL; // Don't pass anything to the callback directly

	// This flight loop callback will be called BEFORE flight model physics
	XPLMCreateFlightLoop_t	SDK210_XPLMCreateFlightLoop_t_ptr = { sizeof(XPLMCreateFlightLoop_t), xplm_FlightLoop_Phase_BeforeFlightModel, XPCFlightLoopCallbackBeforePhysics, NULL };
	SDK210_XPLMTempFlightLoopID = XPLMCreateFlightLoop(&SDK210_XPLMCreateFlightLoop_t_ptr);
	XPLMScheduleFlightLoop(SDK210_XPLMTempFlightLoopID, interval, true);

	// This flight loop callback will be called AFTER flight model physics
	XPLMCreateFlightLoop_t	SDK210_XPLMCreateFlightLoopAfterPhysics_t_ptr = { sizeof(XPLMCreateFlightLoop_t), xplm_FlightLoop_Phase_AfterFlightModel, XPCFlightLoopCallbackAfterPhysics, NULL };
	SDK210_XPLMTempFlightLoopAfterPhysicsID = XPLMCreateFlightLoop(&SDK210_XPLMCreateFlightLoopAfterPhysics_t_ptr);
	XPLMScheduleFlightLoop(SDK210_XPLMTempFlightLoopAfterPhysicsID, interval, true);

	int xpVer;
	XPLMGetVersions(&xpVer, NULL, NULL);
	
	timer->start(chrono::milliseconds(1000), [=]{
		XPC::MessageHandlers::SendBeacon(XPC_PLUGIN_VERSION, RECVPORT, xpVer);
	});
	
	return 1;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho, int inMessage, void* inParam)
{
	// XPC doesn't have anything useful to say to other plugins, so simply ignore
	// any messages received.
}

float XPCFlightLoopCallbackBeforePhysics(float inElapsedSinceLastCall,
	float inElapsedTimeSinceLastFlightLoop,
	int inCounter,
	void* inRefcon)
{
#if (__APPLE__)
	double diff_t;
#endif

	if (benchmarkingSwitch > 1)
	{
		XPC::Log::FormatLine(LOG_DEBUG, "EXEC", "Cycle time %.6f", inElapsedSinceLastCall);
	}

	bool updatedPos = false;
	int cooldownCounterForUpdatedPos = 0;

	int ops;
	for (ops = 0; ops < OPS_PER_CYCLE; ops++)
	{
		if (benchmarkingSwitch > 0)
		{
#if (__APPLE__)
			start = (double)mach_absolute_time( ) * timeConvert;
#endif
		}

		XPC::Message msg = XPC::Message::ReadFrom(*sock);
		if (msg.GetHead() == "")
		{
			break;
		}
		else if (msg.GetHead() == "POSI") 
		{
			writePosiAfterPhysics = true;
			updatedPos = true;
			cooldownCounterForUpdatedPos = 100;  // for 100 frames the last buffered values will be written 
			msgPOSI.CopyMessage(msg);
			XPC::MessageHandlers::HandleMessage(msgPOSI);
		}
		else if (msg.GetHead() == "DREF")
		{
			if (msg.GetDrefName() == "sim/flightmodel/position/local_vx")
			{
				msgVX.CopyMessage(msg);
				XPC::MessageHandlers::HandleMessage(msgVX);
			}
			else if (msg.GetDrefName() == "sim/flightmodel/position/local_vy")
			{
				msgVY.CopyMessage(msg);
				XPC::MessageHandlers::HandleMessage(msgVY);
			}
			else if (msg.GetDrefName() == "sim/flightmodel/position/local_vz")
			{
				msgVZ.CopyMessage(msg);
				XPC::MessageHandlers::HandleMessage(msgVZ);
			}
			else if (msg.GetDrefName() == "sim/flightmodel/position/local_ax")
			{
				msgAX.CopyMessage(msg);
				XPC::MessageHandlers::HandleMessage(msgAX);
			}
			else if (msg.GetDrefName() == "sim/flightmodel/position/local_ay")
			{
				msgAY.CopyMessage(msg);
				XPC::MessageHandlers::HandleMessage(msgAY);
			}
			else if (msg.GetDrefName() == "sim/flightmodel/position/local_az")
			{
				msgAZ.CopyMessage(msg);
				XPC::MessageHandlers::HandleMessage(msgAZ);
			}
			else if (msg.GetDrefName() == "sim/flightmodel/position/P")
			{
				msgP.CopyMessage(msg);
				XPC::MessageHandlers::HandleMessage(msgP);
			}
			else if (msg.GetDrefName() == "sim/flightmodel/position/Q")
			{
				msgQ.CopyMessage(msg);
				XPC::MessageHandlers::HandleMessage(msgQ);
			}
			else if (msg.GetDrefName() == "sim/flightmodel/position/R")
			{
				msgR.CopyMessage(msg);
				XPC::MessageHandlers::HandleMessage(msgR);
			}
			else
			{
				// Handle all other DREFs
				XPC::MessageHandlers::HandleMessage(msg);
			}
		}
		else
		{
			XPC::MessageHandlers::HandleMessage(msg);
		}

		if (benchmarkingSwitch > 0)
		{
#if (__APPLE__)
			lap = (double)mach_absolute_time( ) * timeConvert;
			diff_t = lap - start;
			XPC::Log::FormatLine(LOG_INFO, "EXEC", "Runtime %.6f", diff_t);
#endif
		}
	}

	// If this frame no POSI updates were received (but POSI was updated before) then write saved last POSI update
	if (!updatedPos && msgPOSI.GetSize() != 0 && cooldownCounterForUpdatedPos)
	{
		writePosiAfterPhysics = true;
		XPC::MessageHandlers::HandleMessage(msgPOSI);

		if (msgVX.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgVX);
		if (msgVY.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgVY);
		if (msgVZ.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgVZ);
		if (msgAX.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgAX);
		if (msgAY.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgAY);
		if (msgAZ.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgAZ);
		if (msgP.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgP);
		if (msgQ.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgQ);
		if (msgR.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgR);
	}

	// when the cooldown counter reaches 0 no old POSI updates are written anymore  
	cooldownCounterForUpdatedPos--;

	// If we have processed the maximum number of requests in a single frame,
	// the socket is probably overloaded. Hopefully this is caused by a
	// transitory event like a long load inside X-Plane that caused us to stop
	// responding to requests for a while. We drop the current socket and
	// re-create it to drop any old packets that have probably already timed
	// out on the client side.
	if (ops == OPS_PER_CYCLE)
	{
		XPC::Log::WriteLine(LOG_WARN, "EXEC", "Cleared UDP Buffer");
		delete sock;
		sock = new XPC::UDPSocket(RECVPORT);
		XPC::MessageHandlers::SetSocket(sock);
	}
	return -1;
}

// During this callback the same POSI update values as in the callback before physics are written again 
float XPCFlightLoopCallbackAfterPhysics(float inElapsedSinceLastCall,
	float inElapsedTimeSinceLastFlightLoop,
	int inCounter,
	void* inRefcon)
{
	if (writePosiAfterPhysics)
	{
		writePosiAfterPhysics = false;
		XPC::MessageHandlers::HandleMessage(msgPOSI);

		if (msgVX.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgVX);
		if (msgVY.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgVY);
		if (msgVZ.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgVZ);
		if (msgAX.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgAX);
		if (msgAY.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgAY);
		if (msgAZ.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgAZ);
		if (msgP.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgP);
		if (msgQ.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgQ);
		if (msgR.GetSize() != 0) XPC::MessageHandlers::HandleMessage(msgR);
	}

	return -1;
}

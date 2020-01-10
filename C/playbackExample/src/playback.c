//Copyright (c) 2013-2018 United States Government as represented by the Administrator of the
//National Aeronautics and Space Administration. All Rights Reserved.
//
//DISCLAIMERS
//    No Warranty: THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY KIND,
//    EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, ANY WARRANTY THAT
//    THE SUBJECT SOFTWARE WILL CONFORM TO SPECIFICATIONS, ANY IMPLIED WARRANTIES OF
//    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY
//    THAT THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT DOCUMENTATION, IF PROVIDED,
//    WILL CONFORM TO THE SUBJECT SOFTWARE. THIS AGREEMENT DOES NOT, IN ANY MANNER, CONSTITUTE AN
//    ENDORSEMENT BY GOVERNMENT AGENCY OR ANY PRIOR RECIPIENT OF ANY RESULTS, RESULTING DESIGNS,
//    HARDWARE, SOFTWARE PRODUCTS OR ANY OTHER APPLICATIONS RESULTING FROM USE OF THE SUBJECT
//    SOFTWARE.  FURTHER, GOVERNMENT AGENCY DISCLAIMS ALL WARRANTIES AND LIABILITIES REGARDING
//    THIRD-PARTY SOFTWARE, IF PRESENT IN THE ORIGINAL SOFTWARE, AND DISTRIBUTES IT "AS IS."
//
//    Waiver and Indemnity: RECIPIENT AGREES TO WAIVE ANY AND ALL CLAIMS AGAINST THE UNITED STATES
//    GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY PRIOR RECIPIENT.  IF
//    RECIPIENT'S USE OF THE SUBJECT SOFTWARE RESULTS IN ANY LIABILITIES, DEMANDS, DAMAGES, EXPENSES
//    OR LOSSES ARISING FROM SUCH USE, INCLUDING ANY DAMAGES FROM PRODUCTS BASED ON, OR RESULTING
//    FROM, RECIPIENT'S USE OF THE SUBJECT SOFTWARE, RECIPIENT SHALL INDEMNIFY AND HOLD HARMLESS THE
//    UNITED STATES GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY PRIOR RECIPIENT,
//    TO THE EXTENT PERMITTED BY LAW.  RECIPIENT'S SOLE REMEDY FOR ANY SUCH MATTER SHALL BE THE
//    IMMEDIATE, UNILATERAL TERMINATION OF THIS AGREEMENT.

#include "playback.h"
#include "chrome.h"

#include "xplaneConnect.h"

#include <stdio.h>
#include <time.h>

#include <Windows.h>

double PCFreq = 0.0;
__int64 CounterStart = 0;

void StartCounter()
{
	LARGE_INTEGER li;
	QueryPerformanceFrequency(&li);
	PCFreq = (li.QuadPart) / 1000.0;

	QueryPerformanceCounter(&li);
	CounterStart = li.QuadPart;
}

double GetCounter()
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return (li.QuadPart - CounterStart) / PCFreq;
}


void record(char* path, int interval, int duration)
{
	FILE* fd = fopen(path, "w");
	int count = duration * 1000 / interval;
	if (!fd)
	{
		displayMsg("Unable to open output file.");
		return;
	}
	if (count < 1)
	{
		displayMsg("Duration is less than one iteration.");
		return;
	}
	displayMsg("Recording...");

	//number of datarefs being requested
	unsigned char numberOfDatarefs = 12;
	float* values[12];
	for (int i = 0; i < numberOfDatarefs; i++)
		values[i] = (float*)malloc(1 * sizeof(float));

	int sizes[12] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }; //allocated size of each item in "values"

	//We are using getDREFs() to request a float value
	const char* drefs[12] = {
		"sim/flightmodel/position/local_vx", // The velocity in local OGL coordinates
		"sim/flightmodel/position/local_vy", // The velocity in local OGL coordinates
		"sim/flightmodel/position/local_vz", // The velocity in local OGL coordinates
		"sim/flightmodel/position/local_ax", // The acceleration in local OGL coordinates
		"sim/flightmodel/position/local_ay", // The acceleration in local OGL coordinates
		"sim/flightmodel/position/local_az", // The acceleration in local OGL coordinates
		"sim/flightmodel/position/P",		 // The roll rotation rates (relative to the flight)
		"sim/flightmodel/position/Q",		 // The pitch rotation rates (relative to the flight)
		"sim/flightmodel/position/R",		 // The yaw rotation rates (relative to the flight)
		"sim/flightmodel/position/P_dot",	 // The roll angular acceleration (relative to the flight)
		"sim/flightmodel/position/Q_dot",	 // The pitch angular acceleration (relative to the flight)
		"sim/flightmodel/position/R_dot"	 // The yaw angular acceleration (relative to the flight)
	};

	XPCSocket sock = openUDP("localhost");

	for (int i = 0; i < count; ++i)
	{
		StartCounter();

		double posi[7];
		int result = getPOSI(sock, posi, 0);
		if (result < 0)
		{
			continue;
		}

		fprintf(fd, "%.10lf, %.10lf, %.10lf, %lf, %lf, %lf, %lf, ",
			posi[0], posi[1], posi[2], posi[3], posi[4], posi[5], posi[6]);

		if (getDREFs(sock, drefs, values, numberOfDatarefs, sizes) < 0)
		{
			printf("An error occured."); //negative return value indicates an error
		}
		else
		{
			fprintf(fd, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
				values[0][0], values[1][0], values[2][0], values[3][0], values[4][0], values[5][0],
				values[6][0], values[7][0], values[8][0], values[9][0], values[10][0], values[11][0]);
		}

		int tmp = interval - (int)(GetCounter() + 0.2); // Round value, but only add .2 instead of .5 because of a measures ~.3 delay bewteen StartCounter() and GetCounter()
		if (tmp < 0)
			tmp = 0;
		Sleep(tmp);
	}
	closeUDP(sock);
	displayMsg("Recording Complete");
}

void playback(char* path, int interval)
{
	FILE* fd = fopen(path, "r");
	if (!fd)
	{
		displayMsg("Unable to open output file.");
		return;
	}
	displayMsg("Starting Playback...");

	XPCSocket sock = openUDP("localhost");

	//number of datarefs being requested
	unsigned char numberOfDatarefs = 12;
	float* values[12];
	for (int i = 0; i < numberOfDatarefs; i++)
		values[i] = (float*)malloc(1 * sizeof(float));

	int sizes[12] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }; //allocated size of each item in "values"

	//We are using getDREFs() to request a float value
	const char* drefs[12] = {
		"sim/flightmodel/position/local_vx", // The velocity in local OGL coordinates
		"sim/flightmodel/position/local_vy", // The velocity in local OGL coordinates
		"sim/flightmodel/position/local_vz", // The velocity in local OGL coordinates
		"sim/flightmodel/position/local_ax", // The acceleration in local OGL coordinates
		"sim/flightmodel/position/local_ay", // The acceleration in local OGL coordinates
		"sim/flightmodel/position/local_az", // The acceleration in local OGL coordinates
		"sim/flightmodel/position/P",		 // The roll rotation rates (relative to the flight)
		"sim/flightmodel/position/Q",		 // The pitch rotation rates (relative to the flight)
		"sim/flightmodel/position/R",		 // The yaw rotation rates (relative to the flight)
		"sim/flightmodel/position/P_dot",	 // The roll angular acceleration (relative to the flight)
		"sim/flightmodel/position/Q_dot",	 // The pitch angular acceleration (relative to the flight)
		"sim/flightmodel/position/R_dot"	 // The yaw angular acceleration (relative to the flight)
	};


	double posi[7];

	while (!feof(fd) && !ferror(fd))
	{
		StartCounter();

		int result = fscanf(fd, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
			&posi[0], &posi[1], &posi[2], &posi[3], &posi[4], &posi[5], &posi[6],
			&values[0][0], &values[1][0], &values[2][0],
			&values[3][0], &values[4][0], &values[5][0],
			&values[6][0], &values[7][0], &values[8][0],
			&values[9][0], &values[10][0], &values[11][0]);
		
		if (result != 19)
			continue;

		sendPOSI(sock, posi, 7, 0);
		sendDREFs(sock, drefs, values, sizes, numberOfDatarefs);

		int tmp = interval - (int)(GetCounter() + 0.2); // Round value, but only add .2 instead of .5 because of a measures ~.3 delay bewteen StartCounter() and GetCounter()
		if (tmp < 0)
			tmp = 0;
		Sleep(tmp);
	}

	closeUDP(sock);
	displayMsg("Playback Complete");
}

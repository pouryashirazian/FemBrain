#include "clpp/StopWatch.h"

#ifdef WIN32

double StopWatch::LIToSecs(LARGE_INTEGER& L)
{
	return ((double)L.QuadPart /(double)frequency.QuadPart);
}
 
StopWatch::StopWatch()
{
	timer.start.QuadPart = 0;
	timer.stop.QuadPart = 0; 
	QueryPerformanceFrequency(&frequency);
}
 
void StopWatch::StartTimer()
{
	QueryPerformanceCounter(&timer.start);
}
 
void StopWatch::StopTimer()
{
	QueryPerformanceCounter(&timer.stop);
}
 
double StopWatch::GetElapsedTime()
{
	LARGE_INTEGER time;
	time.QuadPart = timer.stop.QuadPart - timer.start.QuadPart;
	return 1000 * LIToSecs(time);
}

#endif

#if defined(__linux__) || defined(__APPLE__)

StopWatch::StopWatch()
{
	// timer.start.QuadPart = 0;
	// timer.stop.QuadPart = 0; 
	// QueryPerformanceFrequency(&frequency);
}

void StopWatch::StartTimer( )
{
	start = ClockTime();
}
 
void StopWatch::StopTimer()
{
	end = ClockTime();
}

double StopWatch::GetElapsedTime()
{
	return (end - start);
}

#endif

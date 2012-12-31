#ifndef __CLPP_STOPWATCH_H__
#define __CLPP_STOPWATCH_H__

#ifdef WIN32
#include <windows.h>
 
typedef struct
{
	LARGE_INTEGER start;
	LARGE_INTEGER stop;
} stopWatch;

#endif

#if defined(__linux__) || defined(__APPLE__)
#include <sys/time.h>
#endif
 
class StopWatch
{
private:
#ifdef WIN32
	stopWatch timer;
	LARGE_INTEGER frequency;
	double LIToSecs(LARGE_INTEGER& L);
#endif

#if defined(__linux__) || defined(__APPLE__)
	double start;
	double end;

	double ClockTime()
	{
		struct timeval t;
		gettimeofday(&t, 0);

		return t.tv_sec + t.tv_usec / 1000.0;
	}
#endif

public:
	StopWatch();

	void StartTimer();
	void StopTimer();

	double GetElapsedTime();
};

#endif
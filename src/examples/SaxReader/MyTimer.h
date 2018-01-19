#pragma once

#include <windows.h>

class MyTimer
{
private:
	int _freq;
	LARGE_INTEGER _begin;
	LARGE_INTEGER _end;

public:
	float costTime;            //用时,*1000000 = 微秒, 1秒=1000000

	MyTimer(void)
	{
		LARGE_INTEGER tmp;
		QueryPerformanceFrequency(&tmp);
		_freq = tmp.QuadPart;
		costTime = 0;
	}

	~MyTimer(void)
	{

	}

	void Start()            // 开始计时
	{
		QueryPerformanceCounter(&_begin);
	}

	float End()                // 结束计时
	{
		QueryPerformanceCounter(&_end);
		costTime = ((_end.QuadPart - _begin.QuadPart)*1.0f / _freq);

		return costTime;
	}

	void Reset()            // 计时清0
	{
		costTime = 0;
	}
};
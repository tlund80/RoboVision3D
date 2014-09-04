#include "timer.hpp"

Timer::Timer(bool start_now)
{
	started_at_ = 0;
	paused_at_ = 0;
	paused_ = false;
	started_ = false;
	if (start_now)
		start();
}

double Timer::getTime()
{
	timeval t;
	gettimeofday(&t,NULL);
	return (t.tv_sec + t.tv_usec / 1000000.0);
}

void Timer::pause()
{
	if( paused_ || !started_ )
		return;

	paused_ = true;
	paused_at_ = getTime();
}

void Timer::resume()
{
	if( !paused_)
		return;

	paused_ = false;
	started_at_ += getTime() - paused_at_;
}

void Timer::reset()
{
	paused_ = false;
	started_at_ = getTime();
}

void Timer::start()
{
	started_ = true;
	paused_ = false;
	started_at_ = getTime();
}

void Timer::stop()
{
	started_ = false;
}

double Timer::elapsed_msec()
{
	if (!paused_)
		return (getTime() - started_at_) * 1000.0;
	else
		return (paused_at_ - started_at_) * 1000.0;
}

double Timer::elapsed_sec()
{
	if (!paused_)
		return (getTime() - started_at_);
	else
		return (paused_at_ - started_at_);
}

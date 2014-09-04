#ifndef __TIMER_HPP__
#define __TIMER_HPP__

#include <sys/time.h>
#include <iostream>

class Timer {
public:
	/** Default constructor. */
	Timer(bool start_now = false);
	/** Destructor. */
	virtual ~Timer() {};

	void pause();
	void resume();
	void stop();
	void start();
	void reset();

	double elapsed_msec();
	double elapsed_sec();

	bool IsStarted() { return started_; };
	bool IsStopped() { return !started_; };
	bool IsPaused() { return paused_; };
	bool IsActive() { return !paused_ && started_; };

private:
	double started_at_;
	double paused_at_;
	bool started_;
	bool paused_;
	double getTime();
};

#endif // end of __TIMER_HPP__

#include "timer.h"

void Timer::start(int interval, std::function<void()> task)
{
	// is started, do not start again
	if (_expired == false)
		return;

	// start async timer, launch thread and wait in that thread
	_expired = false;
	std::thread([this, interval, task]() {
		while (!_try_to_expire)
		{	
			task();
			// sleep every interval and do the task again and again until times up
			std::this_thread::sleep_for(
					std::chrono::milliseconds(interval));
		}

		{
			// timer be stopped, update the condition variable expired and wake main thread
			std::lock_guard<std::mutex> locker(_mutex);
			_expired = true;
			_expired_cond.notify_one();
		}
	}).detach();
}

void Timer::startOnce(int delay, std::function<void()> task)
{
	std::thread([delay, task]() {
		std::this_thread::sleep_for(
				std::chrono::milliseconds(delay));
		task();
	}).detach();
}

void Timer::stop()
{
	// do not stop again
	if (_expired)
		return;

	if (_try_to_expire)
		return;

	// wait until timer 
	_try_to_expire = true; // change this bool value to make timer while loop stop
	{
		std::unique_lock<std::mutex> locker(_mutex);
		_expired_cond.wait(locker, [this] {return _expired == true; });

		// reset the timer
		if (_expired == true)
			_try_to_expire = false;
	}
}
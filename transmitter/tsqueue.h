#pragma once
#ifndef __queue__
#define __queue__
#include <queue>
//#include <boost\thread.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
template<typename Data> class tsQueue {
private:
	std::queue<Data> the_queue;
	mutable std::mutex the_mutex;
	std::condition_variable the_condition_variable;
public:
	void push(Data const& data)
	{
		std::unique_lock<std::mutex> mlock(the_mutex);
			the_queue.push(data);
		mlock.unlock();

		the_condition_variable.notify_one();
	};

	bool empty() const
	{
		std::lock_guard<std::mutex> mlock(the_mutex);
		return the_queue.empty();
	};

	void wait_and_pop(Data& popped_value)
	{
		std::unique_lock<std::mutex> mlock(the_mutex);
		//while (the_queue.empty()){
		//	the_condition_variable.wait(mlock);
		//}
		the_condition_variable.wait(mlock, [this](){return the_queue.size() != 0; });
		popped_value = the_queue.front();
		the_queue.pop();
	};

	Data pop()
	{
		std::unique_lock<std::mutex> mlock(the_mutex);
			auto popped_value = the_queue.front();
			the_queue.pop();
		mlock.unlock();
		return popped_value;
	}

	int size(bool debug = false)
	{
		std::lock_guard<std::mutex> mlock(the_mutex);
		return the_queue.size();
	};

	void clear()
	{
		std::unique_lock<std::mutex> mlock(the_mutex);
			std::queue<Data> empty;
			std::swap(the_queue, empty);
		mlock.unlock();
	};

	void swap(std::queue<Data> tmpQueue)
	{
		std::unique_lock<std::mutex> mlock(the_mutex);
			std::swap(the_queue, tmpQueue);
		mlock.unlock();
	};
};

#endif
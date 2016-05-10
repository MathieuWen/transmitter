#pragma once
#ifndef __deque__
#define __deque__
#include <deque>
#include <boost\thread.hpp>


template<typename Data> class tsDeque {
private:
	std::deque<Data> the_queue;
	mutable boost::mutex the_mutex;
	boost::condition_variable the_condition_variable;
public:
	void push_back(Data const& data)
	{
		boost::mutex::scoped_lock lock(the_mutex);
		bool const was_empty = the_queue.empty();
		the_queue.push_back(data);
		lock.unlock();
		if (was_empty){
			the_condition_variable.notify_one();
		}
	}
	void push_front(Data const& data)
	{
		boost::mutex::scoped_lock lock(the_mutex);
		bool const was_empty = the_queue.empty();
		the_queue.push_front(data);
		lock.unlock();
		if (was_empty){
			the_condition_variable.notify_one();
		}
	}

	bool empty() const
	{
		boost::mutex::scoped_lock lock(the_mutex);
		return the_queue.empty();
	}

	void wait_and_popback(Data& popped_value)
	{
		boost::mutex::scoped_lock lock(the_mutex);
		while (the_queue.empty()){
			the_condition_variable.wait(lock);
		}
		popped_value = the_queue.back();
		the_queue.pop_back();
	}

	void wait_and_popfont(Data& popped_value)
	{
		boost::mutex::scoped_lock lock(the_mutex);
		while (the_queue.empty()){
			the_condition_variable.wait(lock);
		}
		popped_value = the_queue.front();
		the_queue.pop_front();
	}

	int size()
	{
		boost::mutex::scoped_lock lock(the_mutex);
		return the_queue.size();
	}

	void clear()
	{
		std::deque<Data> empty;
		std::swap(the_queue, empty);
	}
};

#endif
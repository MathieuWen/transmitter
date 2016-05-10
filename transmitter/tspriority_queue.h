#pragma once
#ifndef __priqueue__
#define __pirqueue__
#include <vector>
#include <functional>
#include <boost\thread.hpp>

// top, push, pop, empty, size
template<typename Data, typename alloc, typename comp> class tsPriQueue {
private:
	std::priority_queue<Data, alloc, comp> the_queue;
	mutable boost::mutex the_mutex;
	boost::condition_variable the_condition_variable;
public:
	void push(Data const& data) {
		boost::mutex::scoped_lock lock(the_mutex);
		bool const was_empty = the_queue.empty();
		the_queue.push(data);
		lock.unlock();

		if (was_empty){
			the_condition_variable.notify_one();
		}
	}

	bool empty() const {
		boost::mutex::scoped_lock lock(the_mutex);
		return the_queue.empty();
	}

	void wait_and_pop(Data& popped_value) {
		boost::mutex::scoped_lock lock(the_mutex);
		while (the_queue.empty()){
			the_condition_variable.wait(lock);
		}
		popped_value = the_queue.top();
		the_queue.pop();
	}

	int size() {
		boost::mutex::scoped_lock lock(the_mutex);
		return the_queue.size();
	}
};

#endif
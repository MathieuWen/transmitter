#pragma once
#ifndef __tsvector__
#define __tsvector__
#include <vector>
#include <boost\thread.hpp>
template<typename Data> class tsVector {
	boost::mutex the_mutex;
	std::vector<Data> my_vector;
public:
	bool push_back(Data const& data)
	{
		boost::lock_guard<boost::mutex> lock(the_mutex);
		my_vector.push_back(data);
		return true;
	};

	bool value(int key, Data& popped_value)
	{
		boost::lock_guard<boost::mutex> lock(the_mutex);
		if (key > my_vector.size()||key<0)
			return false;
		else{
			popped_value = my_vector[key];
			return true;
		}
	}

	bool clear()
	{
		boost::lock_guard<boost::mutex> lock(the_mutex);
		my_vector.clear();
		return true;
	}

	int size()
	{
		boost::lock_guard<boost::mutex> lock(the_mutex);
		return my_vector.size();
	};
};

#endif
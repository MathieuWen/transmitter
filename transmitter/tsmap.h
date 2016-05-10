#pragma once
#ifndef __map__
#define __map__
#include <map>
#include <sstream>
#include <string>
#include <mutex>
#include <condition_variable>

template<typename Data1, typename Data2> 
class tsMap {
private:
	std::map<Data1, Data2> the_map;
	typename std::map<Data1, Data2>::iterator the_it;
	mutable std::mutex the_mutex;
	std::condition_variable the_condition_variable;
public:
	std::map<Data1, Data2> getMap()
	{
		return the_map;
	};

	typename std::map<Data1, Data2>::iterator getIter()
	{
		return the_it;
	};

	bool erase(Data1 const &data1)
	{
		std::lock_guard<std::mutex> mlock(the_mutex);
		the_it = the_map.find(data1);
		if (the_it == the_map.end())
			return false;
		else{
			the_map.erase(the_it);
			return true;
		}
	};

	bool find(Data1 const &data1)
	{
		
		try{
			std::lock_guard<std::mutex> mlock(the_mutex);
			the_it = the_map.find(data1);
			//auto it = the_map.find(data1);
			//return (it != the_map.end()) ? true : false;
			return (the_it == the_map.end()) ? false : true;
		}
		catch (std::bad_alloc& ba){
			cerr << " bytes: Out of memory:" << ba.what() << endl;
			exit(1);
		}
		
	}

	void getpair(Data1 &data1, Data2 &data2) 
	{

		std::lock_guard<std::mutex> mlock(the_mutex);
		for (; the_it != the_map.end();++the_it){
			data1 = the_it->first;
			data2 = the_it->second;
		}
	}

	//Data2 value(Data1 const &data1)
	//{
	//	
	//	// 若找不到則會回傳 0 or \0, if int or char
	//	try{
	//		std::lock_guard<std::mutex> mlock(the_mutex);
	//		return the_map[data1];
	//	}
	//	catch (std::bad_alloc& ba){
	//		cerr << " bytes: Out of memory:" << ba.what() << endl;
	//		exit(1);
	//	}
	//};

	Data2 value(Data1 const &data1)
	{

		// 若找不到則會回傳 0 or \0, if int or char
		try{
			std::lock_guard<std::mutex> mlock(the_mutex);
			the_it = the_map.find(data1);
			if (the_it == the_map.end()){
				return NULL;
			}else
				return the_it->second;
			//return the_map[data1];
		}
		catch (std::bad_alloc& ba){
			cerr << " bytes: Out of memory:" << ba.what() << endl;
			exit(1);
		}
	};

	//void insert(Data1 const &data1, Data2 const&data2)
	//{
	//	
	//	try{
	//		std::lock_guard<std::mutex> mlock(the_mutex);
	//		if (the_map.find(data1) == the_map.end()){
	//			the_map.insert(std::make_pair(data1, data2));
	//		}
	//	}
	//	catch (std::bad_alloc& ba){
	//		cerr << " bytes: Out of memory:" << ba.what() << endl;
	//		exit(1);
	//	}
	//};

	bool insert(Data1 const &data1, Data2 const&data2)
	{

		try{
			std::lock_guard<std::mutex> mlock(the_mutex);
			if (the_map.find(data1) == the_map.end()){
				the_map.insert(std::make_pair(data1, data2));
				return true;
			}
			else{
				return false;
			}
		}
		catch (std::bad_alloc& ba){
			cerr << " bytes: Out of memory:" << ba.what() << endl;
			exit(1);
		}
	};

	void update(Data1 const &data1, Data2 const&data2)
	{
		
		try{
			std::lock_guard<std::mutex> mlock(the_mutex);
			the_map[data1] = data2;
		}
		catch (std::bad_alloc& ba){
			cerr << " bytes: Out of memory:" << ba.what() << endl;
			exit(1);
		}
	};

	bool empty() const
	{
		std::lock_guard<std::mutex> mlock(the_mutex);
		return (this.size()>0)?false:true;
	};

	long long size()
	{
		std::lock_guard<std::mutex> mlock(the_mutex);
		return the_map.size();
	};

	void clear()
	{
		std::lock_guard<std::mutex> mlock(the_mutex);
		the_map.clear();
	};

};

#endif
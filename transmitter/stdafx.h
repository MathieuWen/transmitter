// stdafx.h : �i�b�����Y�ɤ��]�t�зǪ��t�� Include �ɡA
// �άO�g�`�ϥΫo�ܤ��ܧ�
// �M�ױM�� Include �ɮ�
//

//#define _CRT_SECURE_NO_DEPRECATE
#pragma warning(disable:4996)
#pragma once
// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
//#pragma comment (lib, "Mswsock.lib")
//#pragma comment (lib, "AdvApi32.lib")
#pragma comment(lib, "avcodec.lib")
#pragma comment(lib, "avformat.lib")
#pragma comment(lib, "swscale.lib")
#pragma comment(lib, "avutil.lib")
#pragma comment(lib, "SDL.lib")
#pragma comment(lib,"Ws2_32.lib") //Winsock Library
#include "targetver.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>  
#include <iomanip>
#include <cstdio>
#include <tchar.h>
#include <stdexcept>
#include <vector>
#include <algorithm>

#include <boost/property_tree/ptree.hpp>
#include <boost/thread/thread.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "boost/regex.hpp"
#include <boost/lexical_cast.hpp>

#include "tsqueue.h"
#include "tsmap.h"
#include "mtrand.h"
#include "fndefine.h"
#include "windowblock.h"
#include <time.h>
#include <math.h>
#include <ctime>
#include <WinSock2.h>
#include <windows.h>

//#include <ws2tcpip.h>

// TODO:  �b���Ѧұz���{���һݭn����L���Y

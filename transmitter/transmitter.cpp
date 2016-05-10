// transmitter.cpp : 定義主控台應用程式的進入點。
//
//#include "stdafx.h"
//#define _CRT_SECURE_NO_DEPRECATE
//#pragma warning(disable:4996)
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
//#include "targetver.h"
extern "C" {
#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavformat/avio.h>
#include <libavutil/avstring.h>
#include <libavutil/time.h>
#include <SDL.h>
#include <SDL_thread.h>
}
#undef main
#include <stdio.h>
#include <stdlib.h>
#include <iostream>  
#include <iomanip>
#include <cstdio>
#include <tchar.h>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#if 1
	#include <boost/thread/thread.hpp>
	#include <boost/thread/mutex.hpp>
	#include <boost/thread/locks.hpp>
#endif
#include "boost/regex.hpp"
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
#include <math.h>
#include <thread>
#include <mutex>
#define SDL_AUDIO_BUFFER_SIZE 1024
#define MAX_AUDIOQ_SIZE (5 * 16 * 1024)
#define MAX_VIDEOQ_SIZE (5 * 256 * 1024)
#define AV_SYNC_THRESHOLD 0.01
#define AV_NOSYNC_THRESHOLD 10.0
#define FF_ALLOC_EVENT   (SDL_USEREVENT)
#define FF_REFRESH_EVENT (SDL_USEREVENT + 1)
#define FF_QUIT_EVENT (SDL_USEREVENT + 2)
#define VIDEO_PICTURE_QUEUE_SIZE 1
#define AVCODEC_MAX_AUDIO_FRAME_SIZE 192000

#define LAYER1 0
#define LAYER2 1
#define EXPERIMENT 0
#define BUFLEN 1001
#define RATECONTROL 1
#define T_NANOSECONDS 1000000000
#define T_MICROSECONDS 1000000
#define MSB 0xF0 // Most Significant Byte
#define LSB 0x0F // Least Significant Byte
//MESSAGE=0x01, META=0x02, SYMBOL=0x03, WBINFO=0x04, CHKINFO=0x05, RETRS=0x06, TEARDOWN=0x07, PROBE=0X10
#define MESSAGE 0x01 
#define META 0x02
#define SYMBOL 0x03
#define WBINFO 0x04
#define CHKINFO 0x05
#define RETRS 0x06
#define TEARDOWN 0x07
#define ACK 0x08
#define NACK 0x09
#define PROBE 0x10
#define RAW 0x11
#define TCPENABLED 0
using namespace std;

/***********************************
*						Global Variables
***********************************/

boost::property_tree::ptree config;
unsigned short port;
string targetIP;
int WinBSize = 200000; //Window Block Size: 2MB (2000000)
int BlockSize = 1000; // Message Block Size, default: 1KB (1000)
double a = 0.3;
int K = 2000;
double OverHead = 0.3;
double OverHead1 = 0.04;
double OverHead2 = 0.04;
int TrDelay = 1000; // us
int ChannelCodingType = 2;
int startupdelay = 3;
int startupDMSize = 1000;
double totalIFrameBlock = 0.0;
double totalRFrameBlock = 0.0;
double recoveryIFrameBlock = 0.0;
double recoveryRFrameBlock = 0.0;
int totalIframe = 0;
int totalPframe = 0;
int destoryIframe = 0; 
int destoryPframe = 0;
//RobustSolitonDistribution 參數
bool RSDSwitch = false;
double parameter_C = 0.01;
double delta = 0.05;

double w0 = 0.55;
const int LayerCount = 2;
int MBCount = WinBSize / BlockSize; // NO of Message Block: 2K, i.e K
int maxDegree = 66;
int *LayerSize; // Layeri中的Message Block大小 
double *layerWeight;
double WinFirstWeight[2] = { 1.0, 0.0 };
double WinLastWeight[2] = { 0.0, 1.0 };
double loss = 0.0;
int cwnd = 3;		// cwnd: 3
time_t Interval = 10000;  // rate control interval: 10 ms = 10^4 us
double *lossRate;
uint64_t Rateout = 0, tmpRateout = 0;
uint64_t Ratein = 2000000; // 發送速度 1Mbps
double RateAlpha = 1 / 8;
double pktLoss = 0.0;

fd_set Transrfd, Recvrfd;
timeval TransTimeout, RecvTimeout;
std::vector<uint64_t> ProbeDelta, OWD;
double thresholdFS = 0.7;
uint64_t RateEpsilon = 1000000;
uint64_t RateAvailable = 0, avgRateAvailable = 0; //bps
uint64_t avgRateout = 0, tavgRateout = 0;
time_t minRTT = 9999999, SRTT = 0, RTTVAR = 0, RTO = 0, maxRTT = 0;
double RTTalpha = 1 / 8, RTTbeta = 1 / 4;
double FASTalpha = 10, FASTgamma = 0.7;
uint64_t SRout = 0;
//const enum pkType { MESSAGE=0x01, META=0x02, SYMBOL=0x03, WBINFO=0x04, CHKINFO=0x05, RETRS=0x06, TEARDOWN=0x07, PROBE=0X10 };
uint16_t EncoderRepeatSSH = 20;
double *DDT; // Store Initial Degree Distribution Table.
double *DDTT; // Store Degree Probability Table.
//double *LPT; // Layer 機率表
bool encoStart = false;
bool splitDone = false;
bool VideoSplitDone = false;
bool encoDone = false;
bool decoDone = false;
bool chkDone = false;
bool probeDone = false;
bool recvDone = false;
bool recvStart = false;
bool playDone = false;
bool txDone = false;
bool transmitterIsInit = false;
bool transmitterInitspinlock = true;
uint64_t encodesleep = 0;
long long cntSB = 0;
long long int degree1 = 0;
long long int ovcnt = 0;
long long int filesize = 0;
long long int totalWBCnt = 0; // 總共 Window Block的數量
long long int MBIndexnotFind = 0;
long long int lastMBIndex; //最後一個Message Block 的 index
int DecodedThreshold = 999999;
int DecodedThresholds[66] = { 0, 0, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1 };
char outputFileName[20];
int trueR = 0;
int falseR = 0;
int totalPKT = 0;
int recvPKT = 0;
uint16_t CurrentMaxWBIndex = 0; // 目前接收到最大的Window Block編號
uint16_t CurrentMinWBIndex = 0; // 尚在解的最小的Window Block編號
uint16_t CurrentPlayWBIndex = 0; // 目前播放的Window Block (GOP)
string times = boost::lexical_cast<string>(time(NULL));
std::mutex GlobalMutex;
std::mutex mutex_blockIndexSet;
typedef enum{
	NONE = 0,
	RATELESS = 1,
	LAYER_ALIGNED_MULTIPRIORITY_RATELESS = 2
} ChannelCoding;


/***********************************
*						Data Structure
***********************************/


WindowBlock WBPreprocess;

class EncodedSymbol
{
public:
	short int type;
	uint8_t *data;	// 此symbol的內容
	int WBIndex;
	int data_size; // 此symbol的大小, default: 1K
	uint64_t seed;
	int window_index; // 此symbol是由哪個WB所構成, 內容為該WB
	// 此symbol是由哪個MessageBlock所構成, 內容為{"該MessageBlock的絕對位置", if degree=1; "-1", otherwise}
	vector<int> block_index;
	int window_size; //此symbol的WindowBlock的實際大小
	int *layer_index; // 各個Layer的起始絕對位置
	int *layer_size; // Layeri中的Message Block大小 
	double *layer_weight; // 各個Layer被挑選中的機率
	int layer_num;
	int degree; // 此symbol的degree
	int threshold;
	int decodedCnt = 0;
	int m; // m=0, 代表為最後的Encoded Symbol, m=1代表還有其它的Encoded Symbol
};

class WindowInfo
{
public:
	int *stream_index;
	uint64_t index;
	uint64_t *pos;
	uint16_t *size;
	uint16_t total;
	int *flags;
	uint64_t *pts;
	uint64_t *dts;
	uint16_t m; // m=0, 代表為最後的Window info, m=1代表還有其它的Window info
};

typedef struct frameinfo
{
	int loc;
	int64_t pos;
	int   flags;
	int   stream_index;
	int   size;
	int64_t pts;
	int64_t dts;
	int m;
} FRAMEINFO;

typedef struct MsgBlock
{
	int64_t loc;
	int size;
} MSGBLOCK;

typedef struct WIPacket
{
	short int type;
	uint64_t sequence;
	uint16_t length;
	uint16_t cwnd;
	time_t timestamp;
	char data[2000];
	uint16_t m;
} WIPACKET;

typedef struct FBPKHDR
{
	short int type;
	uint64_t sequence;
	time_t rcvtime;			//	receiver 收到探測封包的timestamp
	time_t sndtime;			//	transmitter 發送前timestamp
	time_t fdbktime;		//	recevier 回報前的timestamp
	int size;
	double lossrate;
	uint64_t recvrate;
	uint16_t m;
} FBPKHDR;

typedef struct metaPacket
{
	double synpktloss;
	double w0;
	long long int filesize;
	long long int lastMBIndex;
	int BlockSize;
	int totalPKT;
	char filename[1000];
	double alpha;
	double OverHead;
	int WinBSize;
} METAPACKET;

typedef struct WindowsINFO
{
	uint16_t total;
	uint64_t index;
	uint64_t pos[8];
	uint16_t size[8];
	//int stream_index[8];
	//int flags[8];
	//uint64_t pts[8];
	//uint64_t dts[8];
	uint16_t m; // m=0, 代表為最後的Window info, m=1代表還有其它的Window info
} WINDOWSINFO;

class CheckInfo
{
public:
	int WBIndex;
	int WIndex;
	uint64_t UpBound[LayerCount];
	uint64_t LowBound[LayerCount];
	//uint64_t decodeCnt[LayerCount];
	uint16_t isdecoded = 0;
	uint16_t total;
	int window_size;
	int window_index;
	int layer_index[LayerCount]; // 各個Layer的起始絕對位置
	int layer_size[LayerCount]; // Layeri中的Message Block大小 n
	double layer_weight[LayerCount]; // 各個Layer被挑選中的機率
	int layer_num = LayerCount;  // Window內有幾個Layer, 在此只實作2層, default: 2
	uint16_t layer_last;
	uint16_t m = 1;
};

typedef struct ChkInfoPacket
{
	uint64_t WBIndex;
	uint64_t WIndex;
	uint64_t UpBound[2];
	uint64_t LowBound[2];
	uint16_t total;
	uint16_t window_size;
	uint64_t window_index;
	uint64_t layer_index[2]; // 各個Layer的起始絕對位置
	uint16_t layer_size[2]; // Layeri中的Message Block大小 n
	double layer_weight[2]; // 各個Layer被挑選中的機率
	int layer_num;  // Window內有幾個Layer, 在此只實作2層, default: 2
	uint16_t layer_last;
} CHKINFOPACKET;

typedef struct Packet
{
	short int type; // 0: meta, 1: message, 2: Encoded Symbol
	int winseq;
	uint64_t sequence;
	uint64_t maxseq;
	uint64_t minseq;
	int WBIndex;
	uint64_t seed;
	int degree;
	uint16_t size;
	int m;
	char data[BUFLEN];
	uint64_t timestamp;
} PACKET;

typedef struct ProbePacket
{
	uint64_t sequence;
	uint16_t cwnd;
	uint64_t timestamp;
	char data[93];
	int m;
} PROBEPACKET;

tsQueue<WindowBlock> WQ;
tsQueue<EncodedSymbol> SQ;
tsQueue<EncodedSymbol> DQ;
tsQueue<FRAMEINFO> FIQ;
tsQueue<MSGBLOCK> MBQ;
tsQueue<FBPKHDR> receiverTCPFBQ;
tsQueue<WIPacket> transmitterTCPFBQ;
tsQueue<PACKET> PKTQ;
tsQueue<CheckInfo> CKQ;
tsMap<int, uint8_t*> DM;
tsMap<int, uint8_t*> SM;
tsMap<uint64_t, int> calRecoveryM;
tsMap<uint64_t, int> calRecoverySizeM;
typedef vector<CheckInfo> ChkInfo;
ChkInfo CI;
/**************************************************************
Streaming
***************************************************************/
int gop_num = 8; // 預設一組GOP為20個frame 15
double framerate = 20.0;
int viddelay = (int)(1000000.0 / framerate);
int frameBufferSize = 1000000;


enum {
	AV_SYNC_VIDEO_MASTER,
	AV_SYNC_EXTERNAL_MASTER,
};
void videoThread(tsQueue<AVPacket*> *q, int frameBufferSize, AVCodecID c);
typedef struct PacketQueue {
	AVPacketList *first_pkt, *last_pkt;
	int nb_packets;
	int size;
	SDL_mutex *mutex;
	SDL_cond *cond;
} PacketQueue;

typedef struct VideoPicture {
	SDL_Overlay *bmp;
	int width, height; /* source height & width */
	int allocated;
	double pts;
} VideoPicture;

typedef struct VideoState {
	//AVFormatContext *pFormatCtx;
	tsQueue<AVPacket*> *libraryOutput;
	AVCodecID codecID;

	int frameBufferSize;
	int frameBufferCur;
	AVCodecContext  *context;
	int             videoStream;
	int             av_sync_type;
	double          external_clock;
	int64_t         external_clock_time;
	int             seek_req;
	int             seek_flags;
	int64_t         seek_pos;
	double          frame_timer;
	double          frame_last_pts;
	double          frame_last_delay;
	///<pts of last decoded frame / predicted pts of next decoded frame
	double          video_clock;
	///<current displayed pts (different from video_clock if frame fifos are used)
	double          video_current_pts;
	///<time (av_gettime) at which we updated video_current_pts - used to have running video pts
	int64_t         video_current_pts_time;
	//AVStream        *video_st;
	PacketQueue     videoq;
	VideoPicture    pictq[VIDEO_PICTURE_QUEUE_SIZE];
	int             pictq_size, pictq_rindex, pictq_windex;
	SDL_mutex       *pictq_mutex;
	SDL_cond        *pictq_cond;
	SDL_Thread      *parse_tid;
	SDL_Thread      *video_tid;
	int             quit;
} VideoState;


AVFormatContext *pFormatCtx = NULL;
int             videoi, videoStreamIdx;
uint64_t global_video_pkt_pts = AV_NOPTS_VALUE;
AVPacket        packet;
SDL_Surface *screen;
VideoState *global_video_state;

AVPacket flush_pkt;
tsQueue<AVPacket> APKQ;
tsQueue<AVPacket*> *libraryOutput;
tsQueue<AVPacket*> *playerInput;
tsQueue<WindowInfo> WIQ;

void packet_queue_init(PacketQueue *q) {
	std::memset(q, 0, sizeof(PacketQueue));
	q->mutex = SDL_CreateMutex();
	q->cond = SDL_CreateCond();
}

int packet_queue_put(PacketQueue *q, AVPacket *pkt) {

	AVPacketList *pkt1;
	if (pkt != &flush_pkt && av_dup_packet(pkt) < 0) {
		return -1;
	}
	pkt1 = (AVPacketList *)av_malloc(sizeof(AVPacketList));
	if (!pkt1)
		return -1;
	pkt1->pkt = *pkt;
	pkt1->next = NULL;

	SDL_LockMutex(q->mutex);

	if (!q->last_pkt)
		q->first_pkt = pkt1;
	else
		q->last_pkt->next = pkt1;
	q->last_pkt = pkt1;
	q->nb_packets++;
	q->size += pkt1->pkt.size;
	SDL_CondSignal(q->cond);

	SDL_UnlockMutex(q->mutex);
	return 0;
}

static int packet_queue_get(PacketQueue *q, AVPacket *pkt, int block)
{
	AVPacketList *pkt1;
	int ret;

	SDL_LockMutex(q->mutex);

	for (;;) {

		if (global_video_state->quit) {
			ret = -1;
			break;
		}

		pkt1 = q->first_pkt;
		if (pkt1) {
			q->first_pkt = pkt1->next;
			if (!q->first_pkt)
				q->last_pkt = NULL;
			q->nb_packets--;
			q->size -= pkt1->pkt.size;
			*pkt = pkt1->pkt;
			av_free(pkt1);
			ret = 1;
			break;
		}
		else if (!block) {
			ret = 0;
			break;
		}
		else {
			SDL_CondWait(q->cond, q->mutex);
		}
	}
	SDL_UnlockMutex(q->mutex);
	return ret;
}

static void packet_queue_flush(PacketQueue *q) {
	AVPacketList *pkt, *pkt1;

	SDL_LockMutex(q->mutex);
	for (pkt = q->first_pkt; pkt != NULL; pkt = pkt1) {
		pkt1 = pkt->next;
		av_free_packet(&pkt->pkt);
		av_freep(&pkt);
	}
	q->last_pkt = NULL;
	q->first_pkt = NULL;
	q->nb_packets = 0;
	q->size = 0;
	SDL_UnlockMutex(q->mutex);
}

double get_video_clock(VideoState *vidState) {
	double delta;

	delta = (av_gettime() - vidState->video_current_pts_time) / 1000000.0;
	return vidState->video_current_pts + delta;
}

double get_external_clock(VideoState *vidState) {
	return av_gettime() / 1000000.0;
}

double get_master_clock(VideoState *vidState) {
	if (vidState->av_sync_type == AV_SYNC_VIDEO_MASTER) {
		return get_video_clock(vidState);
	}
	else {
		return get_external_clock(vidState);
	}
}

static Uint32 sdl_refresh_timer_cb(Uint32 interval, void *opaque) {
	SDL_Event event;
	event.type = FF_REFRESH_EVENT;
	event.user.data1 = opaque;
	SDL_PushEvent(&event);
	return 0; /* 0 means stop timer */
}

/* schedule a video refresh in 'delay' ms */
static void schedule_refresh(VideoState *is, int delay) {
	SDL_AddTimer(delay, sdl_refresh_timer_cb, is);
}

void video_display(VideoState *vidState) {

	SDL_Rect rect;
	VideoPicture *vp;
	//    AVPicture pict;
	double aspect_ratio;
	int w, h, x, y;
	//    int i;

	vp = &vidState->pictq[vidState->pictq_rindex];
	if (vp->bmp) {
		if (vidState->context->sample_aspect_ratio.num == 0) {
			aspect_ratio = 0;
		}
		else {
			aspect_ratio = av_q2d(vidState->context->sample_aspect_ratio) *
				vidState->context->width / vidState->context->height;
		}
		if (aspect_ratio <= 0.0) {
			aspect_ratio = (float)vidState->context->width /
				(float)vidState->context->height;
		}
		h = screen->h;
		w = ((int)rint(h * aspect_ratio)) & -3;
		if (w > screen->w) {
			w = screen->w;
			h = ((int)rint(w / aspect_ratio)) & -3;
		}
		x = (screen->w - w) / 2;
		y = (screen->h - h) / 2;

		rect.x = x;
		rect.y = y;
		rect.w = w;
		rect.h = h;

		SDL_DisplayYUVOverlay(vp->bmp, &rect);
	}
}

void video_refresh_timer(void *userdata) {

	VideoState *vidState = (VideoState *)userdata;
	VideoPicture *vp;
	double actual_delay, delay, sync_threshold, ref_clock, diff;

	if (vidState->context) {
		if (vidState->pictq_size == 0) {
			schedule_refresh(vidState, 1);
		}
		else {
			vp = &vidState->pictq[vidState->pictq_rindex];

			vidState->video_current_pts = vp->pts;
			vidState->video_current_pts_time = av_gettime();

			delay = vp->pts - vidState->frame_last_pts; /* the pts from last time */
			if (delay <= 0 || delay >= 1.0) {
				/* if incorrect delay, use previous one */
				delay = vidState->frame_last_delay;
			}
			/* save for next time */
			vidState->frame_last_delay = delay;
			vidState->frame_last_pts = vp->pts;


			if (vidState->av_sync_type != AV_SYNC_VIDEO_MASTER) {
				ref_clock = get_master_clock(vidState);
				diff = vp->pts - ref_clock;

				/* Skip or repeat the frame. Take delay into account
				FFPlay still doesn't "know if this vidState the best guess." */
				sync_threshold = (delay > AV_SYNC_THRESHOLD) ? delay : AV_SYNC_THRESHOLD;
				if (fabs(diff) < AV_NOSYNC_THRESHOLD) {
					if (diff <= -sync_threshold) {
						delay = 0;
					}
					else if (diff >= sync_threshold) {
						delay = 2 * delay;
					}
				}
			}

			vidState->frame_timer += delay;
			/* computer the REAL delay */
			actual_delay = vidState->frame_timer - (av_gettime() / 1000000.0);
			if (actual_delay < 0.010) {
				/* Really it should skip the picture instead */
				actual_delay = 0.010;
			}
			schedule_refresh(vidState, (int)(actual_delay * 1000 + 0.5));

			/* show the picture! */
			video_display(vidState);

			/* update queue for next picture! */
			if (++vidState->pictq_rindex == VIDEO_PICTURE_QUEUE_SIZE) {
				vidState->pictq_rindex = 0;
			}
			SDL_LockMutex(vidState->pictq_mutex);
			vidState->pictq_size--;
			SDL_CondSignal(vidState->pictq_cond);
			SDL_UnlockMutex(vidState->pictq_mutex);
		}
	}
	else {
		schedule_refresh(vidState, 100);
	}
}

void alloc_picture(void *userdata) {

	VideoState *vidState = (VideoState *)userdata;
	VideoPicture *vp;

	vp = &vidState->pictq[vidState->pictq_windex];
	if (vp->bmp) {
		// we already have one make another, bigger/smaller
		SDL_FreeYUVOverlay(vp->bmp);
	}
	// Allocate a place to put our YUV image on that screen
	vp->bmp = SDL_CreateYUVOverlay(vidState->context->width,
		vidState->context->height,
		SDL_YV12_OVERLAY,
		screen);
	vp->width = vidState->context->width;
	vp->height = vidState->context->height;

	SDL_LockMutex(vidState->pictq_mutex);
	vp->allocated = 1;
	SDL_CondSignal(vidState->pictq_cond);
	SDL_UnlockMutex(vidState->pictq_mutex);

}

int queue_picture(VideoState *vidState, AVFrame *pFrame, double pts) {

	VideoPicture *vp;
	AVPixelFormat dst_pix_fmt;
	AVPicture pict;

	/* wait until we have space for a new pic */
	SDL_LockMutex(vidState->pictq_mutex);
	while (vidState->pictq_size >= VIDEO_PICTURE_QUEUE_SIZE &&
		!vidState->quit) {
		SDL_CondWait(vidState->pictq_cond, vidState->pictq_mutex);
	}
	SDL_UnlockMutex(vidState->pictq_mutex);

	if (vidState->quit)
		return -1;

	// windex vidState set to 0 initially
	vp = &vidState->pictq[vidState->pictq_windex];

	/* allocate or resize the buffer! */
	if (!vp->bmp ||
		vp->width != vidState->context->width ||
		vp->height != vidState->context->height) {
		SDL_Event event;

		vp->allocated = 0;
		/* we have to do it in the main thread */
		event.type = FF_ALLOC_EVENT;
		event.user.data1 = vidState;
		SDL_PushEvent(&event);

		/* wait until we have a picture allocated */
		SDL_LockMutex(vidState->pictq_mutex);
		while (!vp->allocated && !vidState->quit) {
			SDL_CondWait(vidState->pictq_cond, vidState->pictq_mutex);
		}
		SDL_UnlockMutex(vidState->pictq_mutex);
		if (vidState->quit) {
			return -1;
		}
	}
	/* We have a place to put our picture on the queue */
	/* If we are skipping a frame, do we set this to null
	but still return vp->allocated = 1? */


	if (vp->bmp) {

		SDL_LockYUVOverlay(vp->bmp);

		dst_pix_fmt = PIX_FMT_YUV420P;
		/* point pict at the queue */

		pict.data[0] = vp->bmp->pixels[0];
		pict.data[1] = vp->bmp->pixels[2];
		pict.data[2] = vp->bmp->pixels[1];

		pict.linesize[0] = vp->bmp->pitches[0];
		pict.linesize[1] = vp->bmp->pitches[2];
		pict.linesize[2] = vp->bmp->pitches[1];

		// Convert the image into YUV format that SDL uses
		static struct SwsContext *img_convert_ctx;
		int w = vidState->context->width;
		int h = vidState->context->height;
		if (!img_convert_ctx)
			img_convert_ctx = sws_getContext(w, h, vidState->context->pix_fmt,
			w, h, dst_pix_fmt,
			SWS_X, NULL, NULL, NULL);

		sws_scale(img_convert_ctx, (const uint8_t * const *)pFrame->data,
			pFrame->linesize, 0, h,
			pict.data, pict.linesize);
		SDL_UnlockYUVOverlay(vp->bmp);
		vp->pts = pts;

		/* now we inform our display thread that we have a pic ready */
		if (++vidState->pictq_windex == VIDEO_PICTURE_QUEUE_SIZE) {
			vidState->pictq_windex = 0;
		}
		SDL_LockMutex(vidState->pictq_mutex);
		vidState->pictq_size++;
		SDL_UnlockMutex(vidState->pictq_mutex);
	}
	return 0;
}

double synchronize_video(VideoState *vidState, AVFrame *src_frame, double pts) {

	double frame_delay;

	if (pts != 0) {
		/* if we have pts, set video clock to it */
		vidState->video_clock = pts;
	}
	else {
		/* if we aren't given a pts, set it to the clock */
		pts = vidState->video_clock;
	}
	/* update the video clock */
	frame_delay = av_q2d(vidState->context->time_base);
	/* if we are repeating a frame, adjust clock accordingly */
	frame_delay += src_frame->repeat_pict * (frame_delay * 0.5);
	vidState->video_clock += frame_delay;
	return pts;
}

/* These are called whenever we allocate a frame
* buffer. We use this to store the global_pts in
* a frame at the time it is allocated.
*/
int our_get_buffer(struct AVCodecContext *c, AVFrame *pic, int flags) {
	int ret = avcodec_default_get_buffer2(c, pic, flags);
	uint64_t *pts = (uint64_t *)av_malloc(sizeof(uint64_t));
	*pts = global_video_pkt_pts;
	pic->opaque = pts;
	return ret;
}
void our_release_buffer(struct AVCodecContext *c, AVFrame *pic) {
	if (pic) av_freep(&pic->opaque);
	//avcodec_default_release_buffer(c, pic);
}

int video_thread(void *arg) {
	VideoState *vidState = (VideoState *)arg;
	AVPacket pkt1, *packet = &pkt1;
	//int len1;
	int frameFinished;
	AVFrame *pFrame;
	double pts;

	pFrame = av_frame_alloc();

	while (true) {
		if (packet_queue_get(&vidState->videoq, packet, 1) < 0) {
			// means we quit getting packets
			break;
		}
		if (packet->data == flush_pkt.data) {
			avcodec_flush_buffers(vidState->context);
			continue;
		}
		pts = 0;

		global_video_pkt_pts = packet->pts;

		// Decode video frame
		// len1 = avcodec_decode_video(is->video_st->codec, pFrame, &frameFinished, packet->data, packet->size); -- Deprecated
		avcodec_decode_video2(vidState->context, pFrame, &frameFinished, packet);

		/// Check custom pts value
		if (packet->dts == AV_NOPTS_VALUE
			&& pFrame->opaque
			&& *(uint64_t*)pFrame->opaque != AV_NOPTS_VALUE) {
			pts = (double)(*(uint64_t *)pFrame->opaque);
		}
		else if (packet->dts != AV_NOPTS_VALUE) {
			pts = (double)packet->dts;

		}
		else {
			pts = 0;
		}
		pts *= av_q2d(vidState->context->time_base);

		// Did we get a video frame?
		if (frameFinished) {
			pts = synchronize_video(vidState, pFrame, pts);
			if (queue_picture(vidState, pFrame, pts) < 0) {
				break;
			}
		}
		av_free_packet(packet);
	}


	av_free(pFrame);
	return 0;
}

int stream_component_open(VideoState *vidState, int stream_index) {

	//AVFormatContext *pFormatCtx = is->pFormatCtx;
	//AVCodecContext *codecCtx;
	AVCodec *codec = avcodec_find_decoder(vidState->codecID);
	AVCodecContext* codecCtx = avcodec_alloc_context3(codec);
	avcodec_open2(codecCtx, codec, NULL);
	vidState->context = codecCtx;

	switch (codecCtx->codec_type) {
	case AVMEDIA_TYPE_VIDEO:
		vidState->frame_timer = (double)av_gettime() / 1000000.0;
		vidState->frame_last_delay = 40e-3;
		vidState->video_current_pts_time = av_gettime();

		packet_queue_init(&vidState->videoq);
		vidState->video_tid = SDL_CreateThread(video_thread, vidState);
		codecCtx->get_buffer2 = our_get_buffer;
		//codecCtx->release_buffer = our_release_buffer;

		break;
	default:
		break;
	}
	return 1;
}

int decode_interrupt_cb(void * ctx) {
	return (global_video_state && global_video_state->quit);
}

const AVIOInterruptCB int_cb = { decode_interrupt_cb, NULL };

int decode_thread(void *arg) {

	VideoState *vidState = (VideoState *)arg;
	AVPacket pkt1, *packet = &pkt1;

	int video_index = -1;

	vidState->videoStream = -1;


	global_video_state = vidState;
	stream_component_open(vidState, videoStreamIdx);

	bool waitedPreviously = true;
	while (true) {
		if (vidState->quit) {
			break;
		}

		if (vidState->videoq.size > MAX_VIDEOQ_SIZE) {
			SDL_Delay(10);
			continue;
		}
		vidState->libraryOutput->wait_and_pop(packet);
		packet_queue_put(&vidState->videoq, packet);
	}
	while (!vidState->quit) {
		SDL_Delay(100);
	}

fail:
	{
		SDL_Event event;
		event.type = FF_QUIT_EVENT;
		event.user.data1 = vidState;
		SDL_PushEvent(&event);
	}
	return 0;

}

void ffmpegInit() {
	//std::cout << "check" << std::endl;
	av_register_all();
	if (avformat_open_input(&pFormatCtx, config.get<std::string>("File.name").c_str(), NULL, NULL) != 0)
		exit(1); // Couldn't open file
	if (avformat_find_stream_info(pFormatCtx, NULL) < 0)
		exit(1); // Couldn't find stream information
	//av_dump_format(pFormatCtx, 0, filename.c_str(), 0);
	videoStreamIdx = -1;
	for (videoi = 0; videoi < (int)(pFormatCtx->nb_streams); videoi++){
		if (pFormatCtx->streams[videoi]->codec->codec_type == AVMEDIA_TYPE_VIDEO) { //CODEC_TYPE_VIDEO
			videoStreamIdx = videoi;
			break;
		}
	}

	if (videoStreamIdx == -1)
		exit(1); // Didn't find a video stream
}

void videoThread(tsQueue<AVPacket*> *q, int frameBufferSize, AVCodecID c) {
	//Sleep(2000);
	SDL_Event       event;
	//	double          pts;
	VideoState      *vidState;
	vidState = (VideoState *)av_mallocz(sizeof(VideoState));
	//Sleep(200);
	// Register all formats and codecs
	av_register_all();

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER)) {
		fprintf(stderr, "Could not initialize SDL - %s\n", SDL_GetError());
		exit(1);
	}

	// Make a screen to put our video
#ifndef __DARWIN__
	screen = SDL_SetVideoMode(1024, 768, 0, 0);
	//screen = SDL_SetVideoMode(640, 480, 0, 0);
#else
	screen = SDL_SetVideoMode(1024, 768, 24, 0);
	//screen = SDL_SetVideoMode(640, 480, 24, 0);
#endif
	if (!screen) {
		fprintf(stderr, "SDL: could not set video mode - exiting\n");
		exit(1);
	}

	if (ChannelCodingType == LAYER_ALIGNED_MULTIPRIORITY_RATELESS)
		SDL_WM_SetCaption("ALCC", 0);
	else
		SDL_WM_SetCaption("NON", 0);

	vidState->libraryOutput = q;
	vidState->frameBufferSize = frameBufferSize;
	vidState->frameBufferCur = 0;

	vidState->codecID = c;

	vidState->pictq_mutex = SDL_CreateMutex();
	vidState->pictq_cond = SDL_CreateCond();
	schedule_refresh(vidState, 40);
	vidState->av_sync_type = AV_SYNC_VIDEO_MASTER;
	vidState->parse_tid = SDL_CreateThread(decode_thread, vidState);
	if (!vidState->parse_tid) {
		av_free(vidState);
		return;
	}
	av_init_packet(&flush_pkt);
	flush_pkt.data = (uint8_t*) "FLUSH";
	while (true) {
		SDL_WaitEvent(&event);
		switch (event.type) {
		case FF_QUIT_EVENT:
		case SDL_QUIT:
			vidState->quit = 1;
			SDL_Quit();
			exit(0);
			break;
		case FF_ALLOC_EVENT:
			alloc_picture(event.user.data1);
			break;
		case FF_REFRESH_EVENT:
			video_refresh_timer(event.user.data1);
			break;
		default:
			break;
		}
	}
}

int filEnd;
void VideoSplitter()
{
	int frameIndex = 0;
	int flagFrame;
	int RemSize = 0;
	int offset;
	uint8_t* data;
	int FramePosition = 0;
	ofstream LogSplitter;
	LogSplitter.open("splitter.log", ios::ate);

	do {
		flagFrame = av_read_frame(pFormatCtx, &packet);
		if (packet.stream_index == videoStreamIdx){
			FRAMEINFO FIO;
			FIO.dts = packet.dts;
			FIO.flags = packet.flags;
			FIO.pos = packet.pos;
			FIO.pts = packet.pts;
			FIO.stream_index = packet.stream_index;
			FIO.size = packet.size;
			FIO.loc = FramePosition;
			FIO.m = 1;

			int totalDiv = (int)ceil((double)packet.size / BlockSize); // 做frame的資料切割, 存進MAP中
			RemSize = packet.size;

			for (offset = 0; offset < totalDiv; offset++){
				data = new uint8_t[BlockSize + 1];
				std::memset(data, '\0', sizeof(uint8_t)*(BlockSize + 1));
				std::memset(data, 0xFF, sizeof(uint8_t)*(BlockSize));
				if (BlockSize>RemSize){
					std::memcpy(data, packet.data + (offset*BlockSize), sizeof(uint8_t)*RemSize);
				}
				else{
					std::memcpy(data, packet.data + (offset*BlockSize), sizeof(uint8_t)*BlockSize);
				}
				LogSplitter << FramePosition + (offset*BlockSize) << ", ";
				SM.insert(FramePosition + (offset*BlockSize), data);

				RemSize -= BlockSize;
			}
			LogSplitter << endl;
			FramePosition += offset*BlockSize;
			FIQ.push(FIO);
		}
	} while (flagFrame >= 0);
	filEnd = FramePosition - BlockSize;
	filesize = FramePosition;
	VideoSplitDone = true;
	FRAMEINFO FIO;
	FIO.m = 0;
	FIQ.push(FIO);
	LogSplitter.close();
}

void WBCreator()
{
	ofstream LogWB;
	LogWB.open("LogWB.log", ios::ate);
	int WBIndex = 1; // Window Block index, 0:為第一個WB,...
	int L1Pos = 0;
	int L2Pos = 0;
	int filepos = 0;
	int window_index = 0; // WIndow Block Location, Byte Location
	int i;
	for (i = 0; 1; i++){
		WindowBlock WB;
		WB.index = i + 1;
		WB.window_index = filepos;
		WB.block_size = BlockSize; // 期望切割Window內的Message Block大小 (Byte), default: 1K
		L1Pos = filepos;
		WB.overhead = OverHead; // OverHead
		if (i & 1){ // odd
			L2Pos = filepos + (int)LayerSize[LAYER2];
			WB.layer_index = new int[LayerCount]{L2Pos, L1Pos};
			WB.layer_last = LAYER1;
		}
		else{ // even
			L2Pos = filepos + (int)LayerSize[LAYER1];
			WB.layer_index = new int[LayerCount]{L1Pos, L2Pos};
			WB.layer_last = LAYER2;
		}

		WB.layer_size = new int[LayerCount]{LayerSize[LAYER1], LayerSize[LAYER2]};
		WB.block_num = MBCount;	// Window內中有幾個Message Block, default: 2000
		WB.layer_num = LayerCount; // Window內有幾個Layer, 在此只實作2層, default: 2
		WB.layer_weight = (double *)layerWeight; // 各個Layer被挑選中的機率

		//if (SM.find(filepos + WinBSize)){
		//	filepos = filepos + WinBSize;
		//}
		//else if (SM.find(filEnd)){
		//	filepos = filEnd;
		//}

		LogWB << i << ":" << filepos << "," << filEnd << "," << WinBSize << endl;

		if (filepos + WinBSize > filEnd){
			filepos = filEnd;
		}
		else{
			filepos = filepos + WinBSize;
		}

		WB.window_size = (int)(filepos - L1Pos);
		WB.m = 1;

		//LogWB << i << ":" << filepos << "," << filEnd << endl;

		if (filepos >= filEnd){ // 最後一筆Window Block
			WindowBlock WBlast;
			WBlast.block_size = BlockSize;
			WBlast.overhead = OverHead;
			WBlast.layer_index = new int[LayerCount]{WB.layer_index[LAYER1], WB.layer_index[LAYER2]};
			WBlast.block_num = MBCount;
			WBlast.layer_num = LayerCount;

			if (i & 1){ //odd
				WB.layer_size[LAYER1] = WB.window_size - WB.layer_size[LAYER2]; //重新調整最後一筆WB Layer2的Layer Size
				WBlast.layer_size = new int[LayerCount]{WB.layer_size[LAYER1], 0};
				WBlast.window_size = WB.layer_size[LAYER1];
				WBlast.layer_weight = new double[LayerCount]{1.0, 0.0};
				WBlast.window_index = WB.layer_index[LAYER1];
			}
			else{
				WB.layer_size[LAYER2] = WB.window_size - WB.layer_size[LAYER1]; //重新調整最後一筆WB Layer1的Layer Size
				WBlast.layer_size = new int[LayerCount]{0, WB.layer_size[LAYER2]};
				WBlast.window_size = WB.layer_size[LAYER2];
				WBlast.layer_weight = new double[LayerCount]{0.0, 1.0};
				WBlast.window_index = WB.layer_index[LAYER2];
			}
			WBlast.m = 0;
			WBlast.index = WB.index + 1;
			WQ.push(WB);
			WQ.push(WBlast);


			CheckInfo CKI;
			CKI.window_index = WB.window_index;
			CKI.window_size = WB.window_size;
			CKI.WBIndex = WB.index;
			CKI.WIndex = WB.index;
			memcpy(CKI.layer_weight, WB.layer_weight, sizeof(double)*LayerCount);
			memcpy(CKI.layer_index, WB.layer_index, sizeof(int)*LayerCount);
			memcpy(CKI.layer_size, WB.layer_size, sizeof(int)*LayerCount);
			CI.push_back(CKI);

			CheckInfo CKIlast;
			CKIlast.window_index = WBlast.window_index;
			CKIlast.window_size = WBlast.window_size;
			CKIlast.WBIndex = WBlast.index;
			CKIlast.WIndex = WBlast.index;
			memcpy(CKIlast.layer_weight, WBlast.layer_weight, sizeof(double)*LayerCount);
			memcpy(CKIlast.layer_index, WBlast.layer_index, sizeof(int)*LayerCount);
			memcpy(CKIlast.layer_size, WBlast.layer_size, sizeof(int)*LayerCount);
			CI.push_back(CKIlast);
			break; // 若File descriptor超過檔案結尾的位置表示此WB已經是最後一筆
		}
		else{
			if (i == 0){ // 第一筆Window Block
				WindowBlock WBfirst = WB;
				WBfirst.index = i;
				WBfirst.window_size = WB.layer_size[LAYER1];
				WBfirst.layer_size = new int[LayerCount]{WB.layer_size[LAYER1], 0};
				WBfirst.layer_weight = new double[LayerCount]{1.0, 0.0};
				WQ.push(WBfirst);

				CheckInfo CKIfirst;
				CKIfirst.window_index = WBfirst.window_index;
				CKIfirst.window_size = WBfirst.window_size;
				CKIfirst.WBIndex = WBfirst.index;
				CKIfirst.WIndex = WBfirst.index;
				memcpy(CKIfirst.layer_weight, WBfirst.layer_weight, sizeof(double)*LayerCount);
				memcpy(CKIfirst.layer_index, WBfirst.layer_index, sizeof(int)*LayerCount);
				memcpy(CKIfirst.layer_size, WBfirst.layer_size, sizeof(int)*LayerCount);
				CI.push_back(CKIfirst);
			}

			WQ.push(WB);

			CheckInfo CKI;
			CKI.window_index = WB.window_index;
			CKI.window_size = WB.window_size;
			CKI.WBIndex = WB.index;
			CKI.WIndex = WB.index;
			memcpy(CKI.layer_weight, WB.layer_weight, sizeof(double)*LayerCount);
			memcpy(CKI.layer_index, WB.layer_index, sizeof(int)*LayerCount);
			memcpy(CKI.layer_size, WB.layer_size, sizeof(int)*LayerCount);
			CI.push_back(CKI);
		}

		// 將File descriptor指回Layer 1的結尾, i.e. 此Window的結尾-Layer 2的大小
		filepos = L2Pos;

	}
	LogWB.close();
}

void checker()
{
	double percent = 100.0;
	long long cnt = 0;
	int thresh = config.get<int>("File.thresh");;
	double t_percent = 0.0;
	static volatile long long  mbcnt;

	while (!DM.size());
	//ofstream outputR;
	//outputR.open("output_recovery.log", ios::app | ios::in | ios::out);
	do {
		mbcnt = std::ceil((double)filesize / BlockSize);
		//mbcnt = SM.size();
		if (DM.size()){
			percent = (double)DM.size() / (double)mbcnt;
		}
		else{
			percent = 0;
		}

		if ((t_percent == percent) &&  1){ //  || recvDone //playDone
			cnt++;
		}
		else{
			cnt = 0;
		}
			
		//std::cout.fill('#');
		//std::cout << "SQ:";
		//std::cout << setw(5) << SQ.size();
		std::cout << "PKT:";
		std::cout << recvPKT;
		std::cout << "\tDQ:";
		std::cout << setw(5) << DQ.size();
		std::cout << "\tDM:";
		std::cout << setw(5) << DM.size();
		std::cout << "\tMBCnt:";
		std::cout << setw(5) << mbcnt;
		std::cout << "\t%:";
		std::cout << percent << std::endl;
		//system("cls");
		//if (percent >= 1 || DQ.empty() || cnt>thresh)
		//	break;

		t_percent = percent;
		std::this_thread::sleep_for(std::chrono::microseconds(T_MICROSECONDS));
	} while (cnt < thresh);

	chkDone = true;


	int filepos, fileRem, fileEnd;
	long long total = SM.size();
	double layer1Cnt = 0.0, layer1Total = 0.0, layer2Cnt = 0.0, layer2Total = 0.0;
	int LWBound;
	int L1Limit = (int)LayerSize[LAYER1];
	fileEnd = (int)filesize;
	double succ = 0.0, fail = 0.0, Dmiss = 0.0, Smiss = 0.0;
	for (filepos = LWBound = 0, fileRem = filesize, L1Limit = (int)LayerSize[LAYER1]; filepos < fileEnd; filepos += BlockSize, fileRem -= BlockSize, L1Limit = (int)LayerSize[LAYER1]){
		try{
			if (filepos >= LWBound + WinBSize)
				LWBound += WinBSize;

			if (filepos < LWBound + L1Limit)
				layer1Total++;
			else
				layer2Total++;

			if (!DM.find(filepos))
				Dmiss++;
			if (!SM.find(filepos))
				Smiss++;

			if (DM.find(filepos)){
				if (filepos < LWBound + L1Limit)
					layer1Cnt++;
				else
					layer2Cnt++;
			}

			if (DM.find(filepos) && SM.find(filepos)){
				if (std::strcmp((char *)DM.value(filepos), (char *)SM.value(filepos))){
					fail += 1;
				}
				else{
					succ += 1;
				}
			}
		}
		catch (const exception &e){
			cerr << "bad:" << e.what() << endl;
		}
	}

	std::cout << "a:" << a << "\te:" << OverHead << "\tw0:" << w0 << endl;
	std::cout << "Data Match:" << succ << ", Data Not Match:" << fail << endl;
	std::cout << "Decoded Symbol Miss:" << Dmiss << ", Referenced Symbol Miss:" << Smiss << ", deg1:" << degree1 << endl;
	std::cout << "LastMessageBlock:" << lastMBIndex << ", trueR:" << trueR << ", falseR:" << falseR << endl;
	std::cout << "L1 Recovery Rate:" << (double)(layer1Cnt / layer1Total) * 100 << "%\tL2 Recovery Rate:" << (double)(layer2Cnt / layer2Total) * 100 << "%" << endl;
	std::cout << "L1 Error Rate:" << (layer1Total - layer1Cnt) / layer1Total << "\tL2 Error Rate:" << (layer2Total - layer2Cnt) / layer2Total << endl;
	
	recoveryIFrameBlock = layer1Cnt;
	recoveryRFrameBlock = layer2Cnt;
	totalIFrameBlock = layer1Total;
	totalRFrameBlock = layer2Total;
	//outputR << (double)(layer1Cnt / layer1Total) * 100 << (double)(layer2Cnt / layer2Total) * 100 << "%" << ",";
	//outputR << (layer1Total - layer1Cnt) / layer1Total << (layer2Total - layer2Cnt) / layer2Total << endl;
	//outputR.close();
	Recorder();
}

bool *isExist = new bool[2000000];
bool frontwindow_index = false;
vector<int> recoveryBlockIndex(int window_index, uint64_t seed, int window_size, int *layer_index, int *layer_size, double *layer_weight, int MBSize = BlockSize)
{
	ofstream pick;
lab_recovery_start:
	int repeat = 10;
	int redie = 0;
	//pick.open("pick.csv", ios::app | ios::in | ios::out);
	vector<int> blockIndexSet;
	int realMBCount = (int)ceil((double)window_size / (double)MBSize);
	//bool *isExist = new bool[window_size];
	std::memset(isExist, false, sizeof(bool)* 2000000);

	//if (window_index == 3000000){
	//	pick << "WS:" << window_size << "," << window_index<<"," << realMBCount << endl;
	//}

	int blockIndex = 0;
	double tempRandNum, p1, p2;
	int seed32 = static_cast<uint32_t>(seed);
	MTRand::MTRand_closed drand(seed32);
	MTRand::MTRand_int32 irand(seed32);
	int degree, blockRandNum;
	int ly1Size, ly2Size, LayRange, lytotalSize;
	double die;
	// 決定Degree
	do {
		//if (redie > repeat)
		//	goto lab_recovery_start;
		++redie;
		die = drand();
		degree = GetDegree(die);	// Degree代表將Message Block做XOR的數目
	} while (degree >= realMBCount);		// 若Degree大於實際的Message Block數就重擲

	ly1Size = (int)floor((double)layer_size[LAYER1] / (double)MBSize);
	ly2Size = (int)floor((double)layer_size[LAYER2] / (double)MBSize);
	lytotalSize = (int)floor(layer_weight[LAYER1] * ly1Size + layer_weight[LAYER2] * ly2Size);

	p1 = (double)layer_weight[LAYER1] / lytotalSize;
	p2 = (double)layer_weight[LAYER2] / lytotalSize;

	for (int dgPtr = 0; dgPtr < degree; ++dgPtr){
		do{
			if (layer_weight[LAYER1] == 0.0 || layer_weight[LAYER2] == 0.0){
				LayRange = realMBCount + 1;
				blockRandNum = irand() % LayRange;
			}
			else{
				tempRandNum = drand();
				if (layer_index[LAYER1] < layer_index[LAYER2]){
					if (tempRandNum < p1*ly1Size){
						blockRandNum = (int)floor(tempRandNum / p1);
					}
					else{
						blockRandNum = (int)floor((tempRandNum - p1*ly1Size) / p2) + ly1Size;
					}
				}
				else{
					if (tempRandNum < p2*ly2Size){
						blockRandNum = (int)floor(tempRandNum / p2);
					}
					else{
						blockRandNum = (int)floor((tempRandNum - p2*ly2Size) / p1) + ly2Size;
					}
				}
			}
		} while (isExist[blockRandNum] || blockRandNum*MBSize > window_size);

		blockIndex = window_index + (int)(blockRandNum*sizeof(uint8_t)*MBSize);

		//if (window_index == 17){
		//	if (layer_weight[LAYER1] = 1.0){
		//		if (!frontwindow_index){
		//			pick << endl;
		//		}
		//	}

		//	frontwindow_index = true;
		//}

			

		//if (window_index >= 11000000){
		//		pick << blockIndex << "," << endl;
		//	
		//}


		blockIndexSet.push_back(blockIndex);
		isExist[blockRandNum] = true;

	}
	//std::sort(blockIndexSet.begin(), blockIndexSet.end());
	//pick.close();
	return blockIndexSet;
}


void nonencoder()
{
	if (ChannelCodingType != NONE) return;
	int blockIndex=0;
	int SQSize = 0;
	do{
		if (SM.find(blockIndex)){
			EncodedSymbol SB;
			SB.degree = 1;
			SB.m = 1;
			SB.type = RAW;
			SB.layer_index = new int[2]{0, 0};
			SB.layer_weight = new double[2]{0.0, 0.0};
			SB.layer_size = new int[2]{0, 0};
			SB.seed = blockIndex;
			SB.WBIndex = 0;
			SB.data = new uint8_t[BlockSize + 1];
			std::memcpy(SB.data, SM.value(blockIndex), sizeof(uint8_t)*BlockSize);
			++SQSize;
			SQ.push(SB);
		}
		blockIndex += BlockSize;
	} while (SQSize < SM.size());
}

void encoder()
{
	if (ChannelCodingType != LAYER_ALIGNED_MULTIPRIORITY_RATELESS) return;
	WindowBlock WB;
	boost::posix_time::ptime now;
	int MBSize, t_overhead, realMBCount;
	static volatile double tempRandNum;
	ofstream overheadRecoder, wbRecoder;
	overheadRecoder.open("overheadRecoder.log", ios::ate);
	wbRecoder.open("wbRecoder.csv", ios::ate);
	int blockIndex = 0;
	int encoCnt = 0;
	wbRecoder << "filEnd:" << filEnd <<",WQSize:"<<WQ.size()<< endl;
	wbRecoder << "index,window_index,window_size,layer_index1,layer_index2,layer_size1,layer_size2,layer_weight1,layer_weight2" << endl;
	do{
		//while (!encoStart && encoCnt > 0);
		++encoCnt;
		WQ.wait_and_pop(WB);
		//while (!WQ.try_pop(WB));
		//MBSize = WB.block_size; //預估Message Block Size, default: 1K
		MBSize = BlockSize;

		realMBCount = (int)ceil((double)WB.window_size / (double)MBSize); //計算實際Message Block的數量

		{
			std::lock_guard<std::mutex> mlock(GlobalMutex);
			double ovhead = WB.overhead;
			if (pktLoss > 0.0){
				ovhead = (1.0 + WB.overhead) / (1.0 - pktLoss) - 1.0;
			}

			t_overhead = (int)floor((1.0 + ovhead)*realMBCount / WB.layer_num);

			overheadRecoder << ovhead << ", " << pktLoss << endl;
		}

		wbRecoder << WB.index << "," << WB.window_index << "," << WB.window_size << "," << WB.layer_index[LAYER1] << "," << WB.layer_index[LAYER2] << "," << WB.layer_size[LAYER1] << "," << WB.layer_size[LAYER2] << "," << WB.layer_weight[LAYER1] << "," << WB.layer_weight[LAYER2] << endl;
#if 1
		for (int i = 0; i < t_overhead; i++){
			EncodedSymbol SB;
			//SB.layer_num = WB.layer_num;
			SB.type = SYMBOL;
			SB.window_index = WB.window_index;
			SB.WBIndex = WB.index;
			SB.layer_num = WB.layer_num;
			SB.layer_index = new int[WB.layer_num]{WB.layer_index[LAYER1], WB.layer_index[LAYER2]};
			SB.layer_size = new int[WB.layer_num]{WB.layer_size[LAYER1], WB.layer_size[LAYER2]};
			SB.layer_weight = new double[WB.layer_num]{WB.layer_weight[LAYER1], WB.layer_weight[LAYER2]};
			SB.seed = static_cast<uint64_t>(boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds() + i);
			//SB.seed = i;
			SB.m = 1;
			SB.window_size = WB.window_size;

			uint8_t *tdata = new uint8_t[MBSize + 1];
			uint8_t *data = new uint8_t[MBSize + 1];
			//vector<uint64_t> tmpBlockIndex = recoveryBlockIndex(SB.window_index, SB.seed, SB.window_size, SB.layer_index, SB.layer_size, SB.layer_weight);
			std::unique_lock<std::mutex> mlock(mutex_blockIndexSet);
			SB.block_index = recoveryBlockIndex(SB.window_index, SB.seed, SB.window_size, SB.layer_index, SB.layer_size, SB.layer_weight);
			mlock.unlock();

			int dgPtr = 0;
			std::memset(data, '\0', sizeof(uint8_t)*(MBSize + 1));
			for (std::vector<int>::iterator it = SB.block_index.begin(); it != SB.block_index.end(); ++it){
				std::memset(tdata, '\0', sizeof(uint8_t)*(MBSize + 1));
				std::memcpy(tdata, SM.value(*it), sizeof(uint8_t)*MBSize);
				if (dgPtr == 0){
					// 若為第一個Message Block則直接複製即可
					std::memcpy(data, tdata, sizeof(uint8_t)*MBSize);
				}
				else{
					// 與前一個Message Block做XOR
					for (int x = 0; x < MBSize; x++){
						data[x] ^= tdata[x];
					}
				}
				++dgPtr;
			}

			//SB.block_index.swap(blockIndexSet);
			SB.degree = SB.block_index.size();
			SB.data = new uint8_t[MBSize + 1];
			std::memset(SB.data, '\0', sizeof(uint8_t)*(MBSize + 1));
			std::memcpy(SB.data, data, sizeof(uint8_t)*MBSize);
			SB.data_size = (int)MBSize;
			//std::sort(SB.block_index.begin(), SB.block_index.end());

			SQ.push(SB);
			delete[] data, tdata;
		} // End of Loop (Offset)
#endif

		encoStart = false;
	} while (WQ.size()>0); // End of Loop (while)
	encoDone = true;
	overheadRecoder.close();
	wbRecoder.close();
	//encode.close();
	//encoderSeedRecorder.close();
	//recoverySeedRecorder.close();
}

WSAData wsaData;
SOCKADDR_IN localhostTCP; //宣告 socket 位址資訊(不同的通訊,有不同的位址資訊,所以會有不同的資料結構存放這些位址資訊)
//建立 socket
SOCKET sListen; //listening for an incoming connection
SOCKET sCLient; //operating if a connection was found
SOCKET sock_sink;
SOCKADDR_IN localhostUDP;
int iSResult;
int socketaddr_len;
int  recv_len;

void receiverInit()
{
	try{
		uint16_t port = config.get<unsigned short>("Network.port");
		if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0){ //用 WSAStartup 開始 Winsocket-DLL
			printf("Failed. Error Code : %d", WSAGetLastError());
		}

		sock_sink = INVALID_SOCKET;
		if ((sock_sink = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR){
			std::cout << "Failed:" << WSAGetLastError() << std::endl;
			std::exit(EXIT_FAILURE);
		}

		localhostUDP.sin_family = AF_INET;
		localhostUDP.sin_addr.s_addr = htonl(INADDR_ANY);
		localhostUDP.sin_port = htons(port);
		recv_len = sizeof(localhostUDP);
		if (::bind(sock_sink, (struct sockaddr *)&localhostUDP, sizeof(struct sockaddr_in)) == SOCKET_ERROR){
			std::cout << "bindverr:" << WSAGetLastError() << std::endl;
			std::exit(EXIT_FAILURE);
		}
		else{
			printf("Sever IP: %s\nListening...\n", inet_ntoa(localhostUDP.sin_addr));
		}
/*
		//設定位址資訊的資料
		localhostTCP.sin_addr.s_addr = inet_addr(config.get<std::string>("Network.targetIP").c_str());
		//localhostTCP.sin_addr.s_addr = inet_addr("127.0.0.1");
		localhostTCP.sin_family = AF_INET;
		localhostTCP.sin_port = htons(port);
		//設定 Listen
		//AF_INET：表示建立的 socket 屬於 internet family
		//SOCK_STREAM：表示建立的 socket 是 connection-oriented socket 
		if ((sListen = ::socket(AF_INET, SOCK_STREAM, NULL)) == INVALID_SOCKET){
			printf("Could not create socket : %d", WSAGetLastError());
		}
		
		// Setup the TCP listening socket
		if ((iSResult = ::bind(sListen, (SOCKADDR*)&localhostTCP, sizeof(localhostTCP))) == SOCKET_ERROR){
			printf("Bind failed with error code : %d", WSAGetLastError());
			closesocket(sListen);
			WSACleanup();
			return;
		}

		iSResult = ::listen(sListen, SOMAXCONN);//SOMAXCONN: listening without any limit  //SOMAXCONN
		if (iSResult == SOCKET_ERROR) {
			printf("listen failed with error: %d\n", WSAGetLastError());
			closesocket(sListen);
			WSACleanup();
			return;
		}
		else{
			printf("Sever IP: %s\nListening...\n", inet_ntoa(localhostTCP.sin_addr));
		}
		sCLient = ::accept(sListen, NULL, NULL); // Accept a client socket
		if (sCLient == INVALID_SOCKET) {
			printf("accept failed with error: %d\n", WSAGetLastError());
			closesocket(sListen);
			WSACleanup();
			return;
		}
		// No longer need server socket

		closesocket(sListen);
*/
		socketaddr_len = sizeof(struct sockaddr_in);
	}
	catch (std::exception &e){
		std::cerr << e.what() << std::endl;
	}
}

bool CKIDataDone = false;
#if 0
void receiverTCP()
{
	try{
		//WindowInfo *WI = new WindowInfo;
		WIPACKET WPKT;
		int recvByte;
		bool WBINFOFirst = true;
		vector<CheckInfo> tmpCHKInfo;
		while (true){
			fflush(stdout);
			if ((recvByte = ::recv(sCLient, (char *)&WPKT, sizeof(WPKT), 0)) > 0){
				if (WPKT.type == WBINFO){
					if (WBINFOFirst){
						WIQ.clear();
						WBINFOFirst = false;
					}
					WindowsINFO WINFO;
					memcpy(&WINFO, (WindowsINFO*)&WPKT.data, sizeof(WindowsINFO));
					WindowInfo WI;
					WI.index = WINFO.index;
					WI.m = WINFO.m;
					WI.total = WINFO.total;
					WI.pos = new uint64_t[WI.total];
					memcpy(WI.pos, WINFO.pos, sizeof(WINFO.pos));
					WI.size = new uint16_t[WI.total];
					memcpy(WI.size, WINFO.size, sizeof(WINFO.size));
					WIQ.push(WI);
				}
				else if (WPKT.type == CHKINFO){
					//cout << "CHKINFO" << endl;
					CHKINFOPACKET chkinfoPkt;
					memcpy(&chkinfoPkt, (CHKINFOPACKET*)&WPKT.data, sizeof(CHKINFOPACKET));
					if (WPKT.m && !chkinfoPkt.WBIndex){
						vector <CheckInfo>().swap(tmpCHKInfo);
						//vector <CheckInfo>().swap(CI);
					}
					CheckInfo CKI;
					CKI.WBIndex = chkinfoPkt.WBIndex;
					CKI.WIndex = chkinfoPkt.WIndex;
					//CKI.total = chkinfoPkt.total;
					CKI.window_size = chkinfoPkt.window_size;
					CKI.window_index = chkinfoPkt.window_index;
					CKI.layer_num = chkinfoPkt.layer_num;
					//CKI.layer_last = chkinfoPkt.layer_last;
					//CKI.isdecoded = 0;
					CKI.m = WPKT.m;

					std::copy(chkinfoPkt.layer_weight, chkinfoPkt.layer_weight + 2, CKI.layer_weight);
					//std::copy(chkinfoPkt.LowBound, chkinfoPkt.LowBound + 2, CKI.LowBound);
					//std::copy(chkinfoPkt.UpBound, chkinfoPkt.UpBound + 2, CKI.UpBound);
					std::copy(chkinfoPkt.layer_index, chkinfoPkt.layer_index + 2, CKI.layer_index);
					std::copy(chkinfoPkt.layer_size, chkinfoPkt.layer_size + 2, CKI.layer_size);
					//CKI.decodeCnt[LAYER1] = CKI.decodeCnt[LAYER2] = 0;

					tmpCHKInfo.push_back(CKI);
					if (!CKI.m){
						CI.swap(tmpCHKInfo);
						CKIDataDone = true;
					}
				}
				else if (WPKT.type == RETRS){
					PACKET pkt;
					memcpy(&pkt, (PACKET *)&WPKT.data, sizeof(PACKET));
					if (!DM.find(pkt.seed)){
						uint8_t *data = new uint8_t[BlockSize + 1];
						std::memset(data, '\0', sizeof(uint8_t)*(BlockSize + 1));
						std::memcpy(data, (uint8_t*)pkt.data, sizeof(uint8_t)*(BlockSize));
						DM.insert(pkt.seed, data);
					}
				}
				else if (WPKT.type == META){
					METAPACKET MPK;
					memcpy(&MPK, (METAPACKET*)&WPKT.data, sizeof(METAPACKET));
					{
						std::lock_guard<std::mutex> mlock(GlobalMutex);
						loss = MPK.synpktloss;
						w0 = MPK.w0;
						filesize = MPK.filesize;
						lastMBIndex = MPK.lastMBIndex;
						BlockSize = MPK.BlockSize;
						totalPKT = MPK.totalPKT;
						WinBSize = MPK.WinBSize;
						a = MPK.alpha;
						OverHead = MPK.OverHead;
						memcpy(layerWeight, new double[2]{w0, (1 - w0)}, sizeof(layerWeight));
						memcpy(LayerSize, new int[2]{(int)round(WinBSize*a), (int)round(WinBSize*(1 - a))}, sizeof(LayerSize));
						memcpy(&lossRate, new double[2]{(1.0 - loss), loss}, sizeof(lossRate));
						recvStart = true;
					}

					FBPKHDR FB;
					FB.type = META | PROBE;
					FB.sequence = 0x00;
					FB.sndtime = WPKT.timestamp;
					FB.rcvtime = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
					FB.size = sizeof(WIPACKET);
					FB.recvrate = recvByte * 8 * ((double)T_MICROSECONDS / (FB.rcvtime - FB.sndtime));  // bps
					////cout << FB.recvrate << endl;
					//receiverTCPFBQ.push(FB);
					FB.fdbktime = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
					if ((iSResult = ::send(sCLient, (char *)&FB, sizeof(FBPKHDR), 0)) == SOCKET_ERROR) {
						printf("send failed: %d\n", WSAGetLastError());
					}
				}
				else if (WPKT.type == TEARDOWN){
					{
						std::lock_guard<std::mutex> mlock(GlobalMutex);
						memcpy(&totalPKT, &WPKT.data, sizeof(uint64_t));
					}
					std::chrono::milliseconds dura(2000);
					std::this_thread::sleep_for(dura);
					{
						std::lock_guard<std::mutex> mlock(GlobalMutex);
						//recvDone = true;
					}
					
					//break;
				}
				else if (WPKT.type == SYMBOL){
					PACKET pkt;
					memcpy(&pkt, (METAPACKET*)&WPKT.data, sizeof(PACKET));
					EncodedSymbol SB{};
					{
						std::lock_guard<std::mutex> mlock(GlobalMutex);
						//++recvPKT;
					}
					SB.seed = pkt.seed;
					SB.degree = pkt.degree;
					SB.WBIndex = pkt.WBIndex;
					SB.window_index = CI[SB.WBIndex].window_index;
					SB.data_size = BlockSize;
					SB.window_size = CI[SB.WBIndex].window_size;
					SB.layer_index = new int[CI[SB.WBIndex].layer_num]{CI[SB.WBIndex].layer_index[LAYER1], CI[SB.WBIndex].layer_index[LAYER2]};
					SB.layer_size = new int[CI[SB.WBIndex].layer_num]{CI[SB.WBIndex].layer_size[LAYER1], CI[SB.WBIndex].layer_size[LAYER2]};
					SB.layer_weight = new double[CI[SB.WBIndex].layer_num]{CI[SB.WBIndex].layer_weight[LAYER1], CI[SB.WBIndex].layer_weight[LAYER2]};

					std::unique_lock<std::mutex> mlock(mutex_blockIndexSet);
					vector<int> s = recoveryBlockIndex(SB.window_index, SB.seed, SB.window_size, SB.layer_index, SB.layer_size, SB.layer_weight);
						SB.block_index.swap(s);
					mlock.unlock();

					if (SB.degree == 1){
						uint8_t *data = new uint8_t[BlockSize + 1];
						std::memset(data, '\0', sizeof(uint8_t)*(BlockSize + 1));
						std::memcpy(data, (uint8_t*)pkt.data, sizeof(uint8_t)*(BlockSize));
						{
							std::lock_guard<std::mutex> mlock(GlobalMutex);
							++degree1;
						}
						DM.insert(SB.block_index.at(0), data);
					}
					else{
						SB.data = new uint8_t[BlockSize + 1];
						std::memset(SB.data, '\0', sizeof(uint8_t)*(BlockSize + 1));
						std::memcpy(SB.data, (uint8_t*)pkt.data, sizeof(uint8_t)*(BlockSize));
						SB.threshold = DecodedThreshold;
						DQ.push(SB);
					}
				}
			}
			else if (recvByte == 0){
				printf("Connection closed\n");
				break;
			}
			else{
				printf("recv failed: %d\n", WSAGetLastError());
				break;
			}

		}
	}
	catch (std::exception &e){
		std::cerr << e.what() << std::endl;
	}
}
#endif
void receiver()
{
	ofstream receiverRecoder, msgRecorder;
	receiverRecoder.open("receiver.log", ios::ate);
	msgRecorder.open("msgRecorder.log", ios::ate);
#if EXPERIMENT
	ofstream receiverUDP, receiverRecorvy, probeRecoder;
	receiverUDP.open("receiverUDP.log", ios::ate);
	receiverRecorvy.open("receiverRecorvy.log", ios::ate);
	probeRecoder.open("probeRecoder.log", ios::ate);
	try {
		//calRecoveryM.clear();
		//calRecoverySizeM.clear();
		// 初始值設定為 6 秒
		RecvTimeout.tv_sec = 20;
		RecvTimeout.tv_usec = 0;
		int recvByte;
		uint64_t rcvtime = 0, firstrcvtime = 0, ptime = 0;
		uint64_t sndtime;
		uint64_t ack = 0;
		int nRet;
		uint64_t fbseq = 0;
		uint64_t OWD = 0, SOWD = 0;
		int Rcwnd = -1, RcwndCnt = 0;
		int lastwinseq = -1;
		//uint8_t *data = new uint8_t[BlockSize + 1];
		//while (CI.size()<2) break;
		do {
			fflush(stdout);
			FD_ZERO(&Recvrfd);
			FD_SET(sock_sink, &Recvrfd);
			// 設定 Timer 用來防止超時
			if ((nRet = select(0, &Recvrfd, NULL, NULL, &RecvTimeout)) == SOCKET_ERROR){
				std::cout << "recverr:" << WSAGetLastError() << std::endl;
				break;
			}
			//	Timeout; 表示已經逾時, 要回傳duplicated ACK
			else if (nRet == 0){
				FBPKHDR FB;
				FB.sndtime = 0;
				FB.rcvtime = rcvtime;
				FB.type = NACK;
				FB.sequence = ack;
				FB.fdbktime = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
				FB.lossrate = RcwndCnt / Rcwnd;
				FB.size = 0;
				FB.m = 0;
				::sendto(sock_sink, (char *)&FB, sizeof(FB), 0, (struct sockaddr *)&localhostUDP, recv_len);
				RcwndCnt = 0;
				Rcwnd = -1;
			}
			else{
				PACKET pkt;
				if ((recvByte = ::recvfrom(sock_sink, (char *)&pkt, sizeof(PACKET), 0, (SOCKADDR *)&localhostUDP, (int *)&recv_len)) == SOCKET_ERROR){
					std::cout << "recverr:" << WSAGetLastError() << std::endl;
					break;
				}
				else{
					//	有收到封包
					recvStart = true;
					rcvtime = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
					++recvPKT;
					//if (~Rcwnd){
					//	Rcwnd = pkt.maxseq - pkt.minseq + 1;
					//}
					//if (~lastwinseq){
					//	lastwinseq = pkt.winseq;
					//}

					//if (lastwinseq == pkt.winseq){
					//	++RcwndCnt;
					//	ack = pkt.sequence + 1;
					//}
					//else if (lastwinseq < pkt.winseq){
					//	//FBPKHDR FB;
					//	//FB.sndtime = 0;
					//	//FB.rcvtime = rcvtime;
					//	//FB.type = NACK;
					//	//FB.sequence = ack;
					//	//FB.lossrate = cwndCnt / cwnd;
					//	//FB.fdbktime = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
					//	//FB.size = 0;
					//	//FB.m = 0;
					//	//::sendto(sock_sink, (char *)&FB, sizeof(FB), 0, (struct sockaddr *)&localhostUDP, recv_len);
					//	//lastwinseq = pkt.winseq;
					//	//cwndCnt = 1;
					//}
					//else{
					//	// 由於UDP是 not inorder, 所以可能會收到以前的pkt
					//	// Do nothing...
					//}



				}

				///////////////////////////////////////////
				// 收到探測封包
				//////////////////////////////////////////
				if (pkt.type == PROBE){

					FBPKHDR FB;
					FB.sndtime = pkt.timestamp;
					FB.rcvtime = rcvtime;
					FB.type = PROBE;

					if (ptime>0){	// 若不為第一個封包
						if (rcvtime - ptime){
							OWD = rcvtime - ptime;
							FB.recvrate = recvByte * 8 * T_MICROSECONDS / (rcvtime - ptime);
							if (SOWD)
								SOWD = (1 - RTTalpha)*SOWD + RTTalpha*OWD;
							else
								SOWD = OWD;
						}
						else
							FB.recvrate = recvByte * 8 * T_MICROSECONDS;
						//FB.recvrate = 0;

						//receiverRecoder << OWD << "," << SOWD << endl;
					}
					else{
						FB.recvrate = 0;
					}
					probeRecoder << FB.recvrate << endl;
					ptime = rcvtime;
					FB.sequence = pkt.sequence;
					FB.fdbktime = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
					FB.size = recvByte;
					FB.m = pkt.m;
					::sendto(sock_sink, (char *)&FB, sizeof(FB), 0, (struct sockaddr *)&localhostUDP, recv_len);
					RcwndCnt = 0;
					Rcwnd = -1;
					lastwinseq = -1;
					RecvTimeout.tv_sec = 0;
					RecvTimeout.tv_usec = 100000;
					//goto LabSYMBOL;
				}
				else if (pkt.type == RAW){
					ChannelCodingType = NONE;
					uint8_t *data = new uint8_t[BlockSize + 1];
					std::memset(data, '\0', sizeof(uint8_t)*(BlockSize + 1));

					std::memcpy(data, (uint8_t*)pkt.data, sizeof(uint8_t)*(BlockSize));

					{
						std::lock_guard<std::mutex> mlock(GlobalMutex);
						++degree1;
						//cout <<"+:"<< recvPKT << endl;
						
					}
					int block_index2 = static_cast<int>(pkt.seed);
					receiverUDP << block_index2 << endl;
					DM.insert(block_index2, data);
					
				}
				else if (pkt.type == SYMBOL){			
					ptime = 0;
					ChannelCodingType = LAYER_ALIGNED_MULTIPRIORITY_RATELESS;
					PKTQ.push(pkt);
				}
				else if (pkt.type == MESSAGE){

				}
				else if (pkt.type == META){}



				//if (pkt.m == 0){
				//	recvDone = true;
				//	break;
				//}
			}

			
		} while (playDone==false); // End of Loop

		recvDone = true;
	}
	catch (std::exception &e){
		std::cerr << e.what() << std::endl;
	}
	receiverUDP.close();
	//receiverRecorvy.close();
	//probeRecoder.close();
#else
	// Simulation 用
	recvPKT = 0;
	recvStart = true;
	trueR = 0, falseR = 0;
	//boost::mutex the_mutex;
	do{
		EncodedSymbol SB;
		SQ.wait_and_pop(SB);
		//while (!SQ.try_pop(SB));
		{
			std::lock_guard<std::mutex> mlock(GlobalMutex);
			++recvPKT;
		}

		std::unique_lock<std::mutex> mlock(mutex_blockIndexSet);
		//vector<int> BlockIndexSet = recoveryBlockIndex(SB.window_index, SB.seed, SB.window_size, SB.layer_index, SB.layer_size, SB.layer_weight);
		vector<int> BlockIndexSet = recoveryBlockIndex(CI[SB.WBIndex].window_index, SB.seed, CI[SB.WBIndex].window_size, CI[SB.WBIndex].layer_index, CI[SB.WBIndex].layer_size, CI[SB.WBIndex].layer_weight);
		mlock.unlock();
		
		// 將degree為1的Symbol Block取出, 直接將data存入DecodeMap (因為data即為原始data)
		if (SB.degree == 1){
			DM.insert(BlockIndexSet.at(0), SB.data);
			//DM.insert(SB.block_index.at(0), SB.data);
			degree1++;
			//continue;
		}
		else{

			SB.threshold = DecodedThreshold;


			if (SB.block_index == BlockIndexSet){
				trueR++;
			}
			else{
				falseR++;
			}

			SB.block_index.clear();
			SB.block_index.swap(BlockIndexSet);

			//if (CurrentMaxWBIndex < SB.WBIndex)
			//	CurrentMaxWBIndex = SB.WBIndex;
			//if (CurrentMinWBIndex > SB.WBIndex)
			//	CurrentMinWBIndex = SB.WBIndex;

			DQ.push(SB);
		}
		if (SQ.empty() && WQ.empty())
			break;

	} while (true); //|| SQ.size()>recvPKT
	//recvDone = true;
#endif
	receiverRecoder.close();
}

void PKTProcess()
{
	
	do{
		PACKET pkt;
		PKTQ.wait_and_pop(pkt);
		EncodedSymbol SB{};
		SB.seed = pkt.seed;
		SB.degree = pkt.degree;
		SB.WBIndex = pkt.WBIndex;
		SB.window_index = CI[SB.WBIndex].window_index;
		SB.data_size = BlockSize;
		SB.window_size = CI[SB.WBIndex].window_size;
		SB.layer_index = new int[2]{CI[SB.WBIndex].layer_index[LAYER1], CI[SB.WBIndex].layer_index[LAYER2]};
		SB.layer_size = new int[2]{CI[SB.WBIndex].layer_size[LAYER1], CI[SB.WBIndex].layer_size[LAYER2]};
		SB.layer_weight = new double[2]{CI[SB.WBIndex].layer_weight[LAYER1], CI[SB.WBIndex].layer_weight[LAYER2]};

		std::unique_lock<std::mutex> mlock(mutex_blockIndexSet);
		vector<int> s = recoveryBlockIndex(SB.window_index, SB.seed, SB.window_size, SB.layer_index, SB.layer_size, SB.layer_weight);
		//vector<int> s = recoveryBlockIndex(CI[SB.WBIndex].window_index, SB.seed, CI[SB.WBIndex].window_size, CI[SB.WBIndex].layer_index, CI[SB.WBIndex].layer_size, CI[SB.WBIndex].layer_weight, BlockSize);
		SB.block_index.swap(s);
		s.clear();
		mlock.unlock();

		if (SB.degree == 1){

			uint8_t *data = new uint8_t[BlockSize + 1];
			std::memset(data, '\0', sizeof(uint8_t)*(BlockSize + 1));
			std::memcpy(data, (uint8_t*)pkt.data, sizeof(uint8_t)*(BlockSize));
			{
				//std::lock_guard<std::mutex> mlock(GlobalMutex);
				++degree1;
			}
			DM.insert(SB.block_index.at(0), data);
			//int tmpCnt = calRecoveryM.value(pkt.WBIndex) + 1;
			//calRecoveryM.update(pkt.WBIndex, tmpCnt);
		}
		else{
			SB.data = new uint8_t[BlockSize + 1];
			std::memset(SB.data, '\0', sizeof(uint8_t)*(BlockSize + 1));
			std::memcpy(SB.data, (uint8_t*)pkt.data, sizeof(uint8_t)*(BlockSize));
			//std::memcpy(SB.data, (uint8_t*)data, sizeof(uint8_t)*(BlockSize));
			SB.threshold = DecodedThreshold;
			DQ.push(SB);
		}
	} while (playDone == false);
}

void receiverdown()
{
	//closesocket(sListen);
	closesocket(sock_sink);
	WSACleanup();
}

#if 0
void receiverTCPFeedBack(){
	FBPKHDR FB;
	do {
		receiverTCPFBQ.wait_and_pop(FB);
		FB.fdbktime = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
		if ((iSResult = ::send(sCLient, (char *)&FB, sizeof(FBPKHDR), 0)) == SOCKET_ERROR) {
			printf("send failed: %d\n", WSAGetLastError());
		}
	} while (true);
}
#endif

void decoder()
{
	EncodedSymbol SB;
	
	int block_index;
	//std::vector<int>::iterator it;
	do{
		DQ.wait_and_pop(SB);
		//std::lock_guard<std::mutex> mlock(mutex_blockIndexSet);

		if (SB.block_index.size()==0 || SB.threshold == 0){
			continue;
		}

		
		block_index = SB.block_index.at(0);

		if (SB.degree > 1){
			std::vector<int> tmp_block_index;
			uint8_t *data = new uint8_t[BlockSize + 1];
			std::memset(data, '\0', sizeof(uint8_t)*(BlockSize + 1));
			std::memcpy(data, SB.data, sizeof(uint8_t)*BlockSize);
			tmp_block_index.clear(); // 清空 block_index 暫存
			

			for (std::vector<int>::iterator it = SB.block_index.begin(); it != SB.block_index.end(); ++it){
				if (DM.find(*it)){ // 若有找到MB, 則代表可消掉
					uint8_t *tdata = new uint8_t[BlockSize + 1];
					std::memset(tdata, '\0', sizeof(uint8_t)*(BlockSize + 1));
					std::memcpy(tdata, DM.value(*it), sizeof(uint8_t)* BlockSize);
					int xLimit = (int)(sizeof(uint8_t)*BlockSize);
					for (int x = 0; x < xLimit; x++){
						data[x] ^= tdata[x];
					}			

					if (SB.block_index.size() == 2){
						int iLimit = (int)(SB.block_index.size());
						for (int i = 0; i < iLimit; i++){
							if (SB.block_index.at(i) != *it){
								if (!DM.find(SB.block_index.at(i))){
									DM.insert(SB.block_index.at(i), data);
									//int tmpCnt = calRecoveryM.value(SB.WBIndex) + 1;
									//calRecoveryM.update(SB.WBIndex, tmpCnt);
								}
								goto lab_decoder_end;
							}
						}
					}

					std::memset(SB.data, '\0', sizeof(uint8_t)*(BlockSize + 1));
					std::memcpy(SB.data, data, sizeof(uint8_t)*BlockSize);

					delete[] tdata;
				}
				else{ // 沒找到該Message Block
					tmp_block_index.push_back(*it);
				}
			} // End of Loop (block_index)

			//SB.block_index.clear();
			SB.degree = tmp_block_index.size();
			if (SB.degree){
				//std::unique_lock<std::mutex> mlock(mutex_blockIndexSet);
				SB.block_index.swap(tmp_block_index);
				SB.decodedCnt = SB.decodedCnt + 1;
				DQ.push(SB);
				//mlock.unlock();
			}

			delete[] data;
		}
		else if (SB.degree == 1){
			if (!DM.find(block_index)){
				DM.insert(block_index, SB.data);
				//int tmpCnt = calRecoveryM.value(SB.WBIndex) + 1;
				//calRecoveryM.update(SB.WBIndex, tmpCnt);
			}
		}

	lab_decoder_end:
		1;
	} while (chkDone == false);
}

bool transmitterStart = false;
bool transmitterARQStart = false;

SOCKET sock_descUDP[4];
SOCKADDR_IN serverUDP;
SOCKADDR_IN serverTCP; //宣告給 socket 使用的 sockadder_in 結構
//設定 socket
SOCKET sConnect;
WSADATA wsaServer;
int slen, addlen;

void transmitterInit()
{
	try{

		//Initialise winsock
		if (WSAStartup(MAKEWORD(2, 2), &wsaServer) != 0){
			printf("Failed: %d", WSAGetLastError());
			std::exit(EXIT_FAILURE);
		}

		//Create a socket
		if ((sock_descUDP[0] = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR){
			printf("socket() failed with error code : %d", WSAGetLastError());
			exit(EXIT_FAILURE);
		}


		//setup address structure
		memset((char *)&serverUDP, 0, sizeof(serverUDP));
		port = config.get<unsigned short>("Network.port");
		targetIP = config.get<std::string>("Network.targetIP");
		serverUDP.sin_family = AF_INET;
		serverUDP.sin_port = htons(port);
		serverUDP.sin_addr.S_un.S_addr = inet_addr(targetIP.c_str());
		slen = sizeof(serverUDP);

		/*
		addlen = sizeof(serverTCP);
		//AF_INET: internet-family
		//SOCKET_STREAM: connection-oriented socket
		if ((sConnect = ::socket(AF_INET, SOCK_STREAM, NULL)) == INVALID_SOCKET){
			printf("Could not create socket : %d", WSAGetLastError());
		}
		//設定 addr 資料
		serverTCP.sin_addr.s_addr = inet_addr(config.get<std::string>("Network.targetIP").c_str());
		serverTCP.sin_family = AF_INET;
		serverTCP.sin_port = htons(config.get<unsigned short>("Network.port"));
		if (::connect(sConnect, (SOCKADDR*)&serverTCP, sizeof(serverTCP))){
			puts("connect error");
		}
		*/

	}
	catch (std::exception &e){
		std::cerr << e.what() << std::endl;
	}
}

void transmitterTCP()
{
	int iResult;
	
	uint64_t sequence = 0;
	time_t timestamp;


	do {
		METAPACKET MPK;
		MPK.synpktloss = loss;
		MPK.w0 = w0;
		MPK.alpha = a;
		MPK.filesize = filesize;
		MPK.lastMBIndex = lastMBIndex;
		MPK.BlockSize = BlockSize;
		MPK.totalPKT = totalPKT;
		memcpy(MPK.filename, config.get<std::string>("File.name").c_str(), sizeof(MPK.filename));
		MPK.OverHead = OverHead;
		MPK.WinBSize = WinBSize;
		WIPACKET WPKT;
		WPKT.type = META;
		WPKT.length = sizeof(METAPACKET);
		WPKT.m = 0;
		WPKT.sequence = sequence++;
		timestamp = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds(); //發送時的時間
		cout << "timestamp:" << timestamp << endl;
		WPKT.timestamp = timestamp;
		memcpy(WPKT.data, &MPK, sizeof(METAPACKET));
		if ((iResult = ::send(sConnect, (char *)&WPKT, sizeof(WPKT), 0)) == SOCKET_ERROR) {
			printf("send failed: %d\n", WSAGetLastError());
			return;
		}
		else{
			while (transmitterInitspinlock);
			std::cout << "Transmitter Start!" << endl;
		}
	} while (!transmitterIsInit);



	while (WIQ.size() < 2);
#if TCPENABLED
	//ChkInfo::iterator ptr;
	CheckInfo CKI;
	bool isdecoded = false;
	while (true){
		CKQ.wait_and_pop(CKI);
		CHKINFOPACKET chkinfoPkt;
		chkinfoPkt.WBIndex = CKI.WBIndex;
		chkinfoPkt.WIndex = CKI.WIndex;
		//chkinfoPkt.total = CKI.total;
		chkinfoPkt.window_size = CKI.window_size;
		chkinfoPkt.window_index = CKI.window_index;
		chkinfoPkt.layer_num = CKI.layer_num;
		//chkinfoPkt.layer_last = CKI.layer_last;
		//std::copy(CKI.UpBound, CKI.UpBound + 2, chkinfoPkt.UpBound);
		//std::copy(CKI.LowBound, CKI.LowBound + 2, chkinfoPkt.LowBound);
		std::copy(CKI.layer_index, CKI.layer_index + 2, chkinfoPkt.layer_index);
		std::copy(CKI.layer_size, CKI.layer_size + 2, chkinfoPkt.layer_size);
		std::copy(CKI.layer_weight, CKI.layer_weight + 2, chkinfoPkt.layer_weight);

		WIPACKET WPKT;
		WPKT.type = CHKINFO;
		WPKT.length = sizeof(CHKINFOPACKET);
		WPKT.m = CKI.m;
		WPKT.sequence = sequence++;
		timestamp = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
		WPKT.timestamp = timestamp;
		memcpy(WPKT.data, &chkinfoPkt, sizeof(CHKINFOPACKET));

		if ((iResult = ::send(sConnect, (char *)&WPKT, sizeof(WPKT), 0)) == SOCKET_ERROR) {
			printf("send failed: %d\n", WSAGetLastError());
		}
		else{
			std::cout << "$";
			if (!CKI.m)
				break;
			std::this_thread::sleep_for(std::chrono::microseconds(20));
		}
	}

	WindowInfo WI;
	while(true){
		WIQ.wait_and_pop(WI);
		WindowsINFO WINFO;
		WINFO.m = WI.m;
		WINFO.total = WI.total;
		WINFO.index = WI.index;
		memcpy(WINFO.pos, WI.pos, sizeof(WINFO.pos));
		memcpy(WINFO.size, WI.size, sizeof(WINFO.size));

		WIPACKET WPKT;
		WPKT.type = WBINFO;
		WPKT.length = sizeof(WindowsINFO);
		WPKT.m = WINFO.m;
		WPKT.sequence = sequence++;
		timestamp = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
		WPKT.timestamp = timestamp;
		memcpy(WPKT.data, &WINFO, sizeof(WindowsINFO));

		if ((iResult = ::send(sConnect, (char *)&WPKT, sizeof(WPKT), 0)) == SOCKET_ERROR) {
			printf("send failed: %d\n", WSAGetLastError());
		}
		else{
			std::cout << "*";
			if (!WI.m)
				break;
			std::this_thread::sleep_for(std::chrono::microseconds(20));
		}
	}

#endif
	transmitterStart = true;

	while (!txDone);
	WIPACKET WPKT;
	WPKT.type = TEARDOWN;
	WPKT.length = 1;
	WPKT.m = 0;

	{
		std::lock_guard<std::mutex> mlock(GlobalMutex);
		memcpy(WPKT.data, (char *)&totalPKT, sizeof(uint64_t));
	}
	WPKT.sequence = sequence++;
	timestamp = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
	WPKT.timestamp = timestamp;

	//memcpy(WPKT.data, 0x00, sizeof(WPKT.data));
	if ((iResult = ::send(sConnect, (char *)&WPKT, sizeof(WPKT), 0)) == SOCKET_ERROR) {
		printf("send failed: %d\n", WSAGetLastError());
	}
	else{
		std::cout << "Done!, totalPKT:" << totalPKT << endl;
	}

}

int probingtime = 20;
void transmitterProbe(ofstream &transmitterProbelog)
{
	try {
		uint64_t probeseq = 0;
		time_t sleeptime = 0;
		//PROBEPACKET probe;
		PACKET probe;

		for (int i = 0; i < probingtime; i++){
			/************************/
			//EncodedSymbol SB = SQ.pop();
			//probe.minseq = 0;
			//probe.seed = SB.seed;
			//probe.degree = SB.degree;
			//probe.WBIndex = SB.WBIndex;
			//probe.maxseq = 0;
			//probe.minseq = 0;
			//probe.size = sizeof(SB.data);
			//std::memset(probe.data, '\0', BUFLEN);
			//std::memcpy(probe.data, (char*)SB.data, sizeof(SB.data));
			//probe.maxseq = probingtime - 1;
			/*************************/
			probe.sequence = i;
			probe.type = PROBE;
			probe.timestamp = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
			probe.m = (i == probingtime - 1) ? 0 : 1;
			if (::sendto(sock_descUDP[0], (char *)&probe, sizeof(probe), 0, (struct sockaddr *)&serverUDP, slen) == SOCKET_ERROR){
				std::cout << "probe err:" << WSAGetLastError() << endl;
				std::exit(EXIT_FAILURE);
			}
			else{
				SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY | FOREGROUND_RED | FOREGROUND_GREEN);
				std::cout << "#";
				SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
			}
			if (Ratein){ //&& pktLoss < 0.70
				//std::lock_guard<std::mutex> mlock(GlobalMutex);
				long double t_time = sizeof(PACKET)* 8.0;
				sleeptime = t_time / Ratein * T_MICROSECONDS;
			}
			else
				sleeptime = 10000; //10000
			//std::this_thread::sleep_for(std::chrono::microseconds(sleeptime));
			std::this_thread::sleep_for(std::chrono::microseconds(10000));

		}
		transmitterProbelog << "Ratein: " << Ratein << ", cwnd: " << cwnd << ", sleeptime: " << sleeptime << endl;
		while (!probeDone);
	}
	catch (std::exception &e){
		std::cerr << e.what() << std::endl;
	}

}



bool isRetran = false;
std::mutex channel_mutex;
uint64_t cwndStart = 99999, cwndEnd = 0, sequence = 0;
void transmitter()
{
	try{
		std::this_thread::sleep_for(std::chrono::microseconds(20000));
		//ofstream transmitterProbelog, pcketRecoder;
		//transmitterProbelog.open("transmitterProbelog.log", ios::ate);
		//pcketRecoder.open("pcketRecoder.log", ios::ate);
		volatile int piggybackCnt = 1;
		int pgcwnd = 1;
		{
			//std::lock_guard<std::mutex> mlock(GlobalMutex);
			piggybackCnt = cwnd;
			pgcwnd = cwnd;
		}
		//char *data = new char[BUFLEN];		
		volatile time_t ptime;
		volatile time_t ltime;
		uint64_t currentWBIndex = 99999;
		uint64_t minseq = 0, maxseq = minseq + pgcwnd - 1;
		int cwndseq = 0, cwndCnt = 0;

		{
			//std::lock_guard<std::mutex> mlock(GlobalMutex);
			encoStart = true;
		}

		//while (transmitterARQStart);
		//while (!transmitterStart);
#if 1
		do {
			//while (isRetran);
			PACKET pkt;
			EncodedSymbol SB{};
			SQ.wait_and_pop(SB);
			if (SQ.empty()){
				encoStart = true;
			}
			if (cwndEnd < sequence || sequence == 0){
				//std::lock_guard<std::mutex> mlock(GlobalMutex);
				//PACKETGROUP.insert(cwndseq, cwndCnt);
				cwndStart = sequence;
				cwndEnd = sequence + cwnd;
				cwndseq = cwndseq + 1;
				cwndCnt = 0;
			}

			//if (currentWBIndex != SB.WBIndex){
			//	if (currentWBIndex%4==0){
			//		probeDone = false;
			//		std::thread probe(transmitterProbe, std::ref(transmitterProbelog));
			//		probe.join();

			//	}
			//	currentWBIndex = SB.WBIndex;
			//}


			pkt.type = SB.type;
			pkt.degree = SB.degree;

			pkt.sequence = sequence;
			pkt.seed = SB.seed;
			pkt.WBIndex = SB.WBIndex;
			pkt.maxseq = maxseq;
			pkt.minseq = minseq;
			pkt.size = sizeof(SB.data);
			pkt.winseq = cwndseq;
			pkt.m = (SQ.size() == 0 && WQ.size() == 0) ? 0 : 1;
			++cwndCnt;
			std::memset(pkt.data, '\0', BUFLEN);
			std::memcpy(pkt.data, (char*)SB.data, sizeof(uint8_t)*BlockSize);

			pkt.timestamp = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
			//PACKETMAP.insert(sequence, pkt);

			//pcketRecoder << sequence << endl;
			if (::sendto(sock_descUDP[0], (char *)&pkt, sizeof(pkt), 0, (struct sockaddr *)&serverUDP, slen) == SOCKET_ERROR){
				std::cout << "senderr:" << WSAGetLastError() << endl;
				std::exit(EXIT_FAILURE);
			}
			else {
				sequence = sequence + 1;
				--piggybackCnt;

				{
					std::lock_guard<std::mutex> mlock(GlobalMutex);
					++totalPKT;
				}

				if (pkt.degree == 1){
					SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_BLUE | FOREGROUND_INTENSITY);
					std::cout << ".";
					SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
				}
				else{
					std::cout << ".";
				}

				std::this_thread::sleep_for(std::chrono::microseconds(TrDelay));//

				//ptime = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
				//while (true){
				//	ltime = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
				//	if ((ltime - ptime) >= TrDelay){
				//		break;
				//	}
				//}
			}

#if 0
			ptime = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
			// busy-waitting idle for transmitter
			while (piggybackCnt <= 0){
				ltime = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
				if ((ltime - ptime) > Interval){
					//boost::lock_guard<boost::mutex> lock(GlobalMutex);
					//std::lock_guard<std::mutex> mlock(GlobalMutex);
					piggybackCnt = pgcwnd;
					pgcwnd = cwnd;
					maxseq = minseq + pgcwnd - 1;
					minseq = sequence;
					break;
				}
			}
#endif
			

			if (SQ.size()==0 ){
				txDone = true;
				break;
			}
		} while (true); //End of Loop (type=SYMBOL)
#endif

		std::cout << "Done!, totalPKT:" << totalPKT << ", SM:" << SM.size() << endl;


		//transmitterProbelog.close();
		//pcketRecoder.close();

		
	}
	catch (std::exception &e){
		std::cerr << e.what() << std::endl;
	}


}

void transmitterdown(){
	//closesocket(sConnect);
	closesocket(sock_descUDP[0]);
	WSACleanup();
}
#if 0
/////////////////////////////////////////
//		TCP接收回饋的訊息
/////////////////////////////////////////
void transmitterTCPReceptionist()
{
	try{
		ofstream recoveryRateReocder;
		recoveryRateReocder.open("recoveryRateReocder.log", ios::ate);
		time_t ptime = 0, ltime = 0;
		int iResult;
		int recvByte;
		FBPKHDR FB;
		uint64_t block_index;
		uint64_t sequence = 0;
		time_t timestamp;
		//char *data = new char[BUFLEN];
		while (true){
			fflush(stdout);
			if ((recvByte = ::recv(sConnect, (char *)&FB, sizeof(FBPKHDR), 0)) > 0){
				//if (ltime - ptime > 1000) cwnd = 1;
				//cout << (FB.type&0x0F) << endl;
				ptime = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
				if ((FB.type & 0x0F) == RETRS){

				}
				else if ((FB.type & LSB) == TEARDOWN){
					transmitterdown();
				}

				if ((FB.type & MSB) == PROBE){
					if ((FB.type & 0x0F) == META){
						uint64_t sndtime = FB.sndtime, rcvtime = FB.rcvtime;
						long double durtation = ((long long int)rcvtime - (long long int)sndtime) / T_MICROSECONDS;
						transmitterInitspinlock = false;
						transmitterIsInit = true;
					}


				}

				if (FB.type == META){
					std::lock_guard<std::mutex> mlock(GlobalMutex);
					pktLoss = (1.0 - FB.lossrate / FB.size) * 0.7;
					if (pktLoss >= 1.0){
						pktLoss = 0.0;
					}
					tmpRateout = FB.recvrate;
					recoveryRateReocder << FB.size << "," << FB.lossrate << "," << FB.lossrate / FB.size << endl;
				}

				ltime = ptime;
			}
			else if (recvByte == 0){
				printf("Connection closed\n");
				break;
			}
			else{
				printf("recv failed: %d\n", WSAGetLastError());
				break;
			}

		}
		recoveryRateReocder.close();
	}
	catch (std::exception &e){
		std::cerr << e.what() << std::endl;
	}
}
#endif


uint64_t tRateAvailable = 0;

void transmitterFlowControl()
{
	std::lock_guard<std::mutex> mlock(GlobalMutex);
	FASTalpha = std::ceil((double)RateAvailable / 8 / sizeof(PACKET));
	//FASTalpha = 100;
	tRateAvailable = RateAvailable;
}


int getAvailableBandwidth()
{

	int identity = 0, M = ProbeDelta.size();
	if (M == 0) return 0;
	int flag = 0;
	double FS = 0.0;
	uint64_t time = ProbeDelta[0];
	for (int i = 1; i < M; i++){
		for (int l = 0; l < i; l++){
			if (ProbeDelta[i]>ProbeDelta[l]){
				++identity;
			}
		}
		time += ProbeDelta[i];
	}

	M = (M * (M - 1)) / 2;
	FS = (double)identity / (double)M;
	//cout << endl << FS << ", " << Ratein << ":" << avgRateout << endl;
	if (FS < thresholdFS){ // || (avgRateout - Ratein) >= RateEpsilon
		std::lock_guard<std::mutex> mlock(GlobalMutex);
		RateAvailable = Rateout;
		flag = 1;
	}
	else{
		std::lock_guard<std::mutex> mlock(GlobalMutex);
		Ratein = Rateout;
	}

	return flag;
}

int probeCnt = 0;
uint64_t lastAcked = 999999;

////////////////////////////////////////
//		UDP 回饋機制
///////////////////////////////////////
void TransmitterReceptionist()
{
	try{
		ofstream probe, ackRecoder, nack, lossRecoder;
		probe.open("probe.log", ios::ate);
		ackRecoder.open("ack.log", ios::ate);
		nack.open("nack.log", ios::ate);
		lossRecoder.open("lossRecoder.log", ios::ate);
		TransTimeout.tv_sec = 3;	/* seconds */
		TransTimeout.tv_usec = 0;	/* microseconds */
		FBPKHDR FB;
		int recvByte;
		int nRet;
		uint64_t tmpcwndEnd = 0, tmpcwndStart = 0;
		time_t CurrenTimeStamp = 0, RTT = 0; /* nanoseconds */
		do {
			fflush(stdout);
			FD_ZERO(&Transrfd);
			FD_SET(sock_descUDP[0], &Transrfd);

			if ((nRet = select(0, &Transrfd, NULL, NULL, &TransTimeout)) == SOCKET_ERROR){
				std::cout << "recverr:" << WSAGetLastError() << std::endl;
				break;
			}
			else if (nRet == 0) {	// Timeout; 做一次更新動作
				lossRecoder << "t" << endl;
				std::lock_guard<std::mutex> mlock(GlobalMutex);
				//cwnd = cwnd * 0.5;
				//Ratein = 10000;
				probeDone = true;
			}
			else{

				if ((recvByte = recvfrom(sock_descUDP[0], (char *)&FB, sizeof(FBPKHDR), 0, (SOCKADDR *)&serverUDP, (int *)&slen)) == SOCKET_ERROR){
					std::cout << "recverr:" << WSAGetLastError() << std::endl;
					break;
				}
				else{
					CurrenTimeStamp = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
					//std::lock_guard<std::mutex> mlock(GlobalMutex);

/*
					if (FB.type == ACK) {
						RTT = CurrenTimeStamp - FB.sndtime;
						if (SRTT == 0){
							std::lock_guard<std::mutex> mlock(GlobalMutex);
							SRTT = RTT;
							RTTVAR = RTT / 2;
							RTO = SRTT + 4 * RTTVAR;

						}
						else{
							std::lock_guard<std::mutex> mlock(GlobalMutex);
							RTTVAR = (1 - RTTbeta) * RTTVAR + RTTbeta * std::abs(SRTT - RTT);
							SRTT = (1 - RTTalpha)*SRTT + RTTalpha*RTT;
							RTO = SRTT + 4 * RTTVAR;

						}

						// 求得 Propagation Delay
						if (minRTT > RTT && RTT > 0){
							std::lock_guard<std::mutex> mlock(GlobalMutex);
							minRTT = RTT;
						}


						// 求得 Queuing Delay 的 UpBound, i.e. 瓶頸頻寬的最大容量
						std::lock_guard<std::mutex> mlock(GlobalMutex);
						if (maxRTT < RTT){
							//std::lock_guard<std::mutex> mlock(GlobalMutex);
							maxRTT = RTT;
						}

						int FASTcwnd = 99999;
						if (RTT)
							FASTcwnd = (1 - FASTgamma)*cwnd + FASTgamma*(minRTT / RTT*cwnd + FASTalpha);

						if (2 * cwnd > FASTcwnd && RTT > 0){
							cwnd = FASTcwnd;
						}
						else{
							cwnd = std::ceil(2 * cwnd);
						}

						ackRecoder << cwnd << "," << minRTT << "," << maxRTT << "," << SRTT << "," << FASTalpha << endl;
						TransTimeout.tv_sec = 0;
						TransTimeout.tv_usec = 50000; //100ms

					}
					else if (FB.type == NACK){
						std::lock_guard<std::mutex> mlock(GlobalMutex);
						lossRecoder << FB.sequence << endl;
						cwnd = std::ceil(cwnd * 0.5);
						if (cwnd >= 0){
							cwnd = 1;
						}
						probeDone = true;
					}
					if ((FB.type & MSB) == PROBE || FB.type == PROBE){
						++probeCnt;

						OWD.push_back(FB.rcvtime);  //  (FB.rcvtime - FB.sndtime)

						probe << setw(2) << probeCnt << ": " << Rateout << endl;

						int key = OWD.size();
						if (key >= 2){
							ProbeDelta.push_back(OWD.at(key - 1) - OWD.at(key - 2));
						}

						// 若 packet chain 到結尾時
						if (FB.m == 0 && probeCnt == probingtime){
							//std::lock_guard<std::mutex> mlock(GlobalMutex);
							Rateout = FB.recvrate; // 接收速率

							if (getAvailableBandwidth()){		// 預測可用頻寬
								transmitterFlowControl();		// 調整發送速度
							}
							else{
								//std::lock_guard<std::mutex> mlock(GlobalMutex);
								//FASTalpha = std::ceil((double)tRateAvailable / 8 / sizeof(PACKET));
							}
							cout << endl << "cwnd: " << cwnd << ", RateAvailable:" << RateAvailable / 8 / sizeof(PACKET) << endl;
							probe << "Cwnd: " << cwnd << ", RateAvailable: " << RateAvailable << ", FASTalpha: " << FASTalpha << endl;
							vector <uint64_t>().swap(OWD);
							vector <uint64_t>().swap(ProbeDelta);
							probeCnt = 0;
							Rateout = 0;
							tavgRateout = avgRateout;
							avgRateout = 0;
							probeDone = true;
						}
						else{
							if (tmpRateout > 0)
								Rateout = tmpRateout;
							else
								Rateout = 2000000;
						}

						TransTimeout.tv_sec = 0;
						TransTimeout.tv_usec = 50000; //100ms

					}

*/
				}
			}


			//cout << endl<<"RTT:" << SRTT << endl;
		} while (true);
		ackRecoder.close();
		nack.close();
		probe.close();
		lossRecoder.close();
	}
	catch (std::exception &e){
		std::cerr << e.what() << std::endl;
	}
}

/*
*		使用Binary Search Algorithm來查表
*		Get Degree in Degree Distribution Table by ProbValue.
*/
int GetDegree(double ProbValue)
{
	int lo = 0;
	int hi = MBCount;

	while (lo < hi){
		int mid = (lo + hi) / 2;
		if (DDT[mid] < ProbValue)
			lo = mid + 1;
		else
			hi = mid;
	}
	return (lo < MBCount) ? lo + 1 : MBCount; // Avoid Degree >= DDT_Size + 1.
}

int GetLayer(double ProbValue, double *ProbTable)
{
	int lo = 0;
	int hi = LayerCount - 1;
	double *LPT = LayerProGenerator(ProbTable);
	while (lo < hi){
		int mid = (lo + hi) / 2;
		if (LPT[mid] < ProbValue)
			lo = mid + 1;
		else
			hi = mid;
	}
	return hi;
}

void RobustSolitonDistribution(double* Omega)
{
	double* rho = new double[MBCount];
	double* tau = new double[MBCount];
	double* mu = new double[MBCount];
	double R;
	double beta;
	int tempkR;
	int i;

	R = (float)(parameter_C * log(MBCount / delta)*sqrt((double)MBCount));

	tempkR = (int)ceil(MBCount / R - 0.5);

	for (i = 1; i <= MBCount; i++){
		if (i <= tempkR - 1){
			tau[i - 1] = R / ((double)i*MBCount);
		}
		else if (i == tempkR){
			tau[i - 1] = R*log(R / delta) / MBCount;
		}
		else{
			tau[i - 1] = 0;
		}
	}

	for (i = 1; i <= MBCount; i++){
		if (i == 1){
			rho[i - 1] = 1 / MBCount;
		}
		else{
			rho[i - 1] = 1 / (double)(i*(i - 1));
		}
	}

	for (i = 1, beta = 0; i <= MBCount; i++){
		beta = beta + rho[i - 1] + tau[i - 1];
	}

	for (i = 1; i <= MBCount; i++){
		Omega[i - 1] = mu[i - 1] = (rho[i - 1] + tau[i - 1]) / beta;
	}

	delete[] rho;
	delete[] tau;
	delete[] mu;
}

double *RaptorDistribution(int CumuFlag)
{
	// MBCount: Message Block Count 預設為2000
	double* Omega = new double[MBCount];
	int degree;

	//for (degree = 0; degree < MBCount; degree++){
	//	Omega[degree] = 0.0;
	//}

	memset(Omega, 0.0f, sizeof(double)*MBCount);

	if (RSDSwitch){
		RobustSolitonDistribution(Omega);
		maxDegree = MBCount;
	}
	else{
#if 1
		Omega[0] = 0.007969;
		Omega[1] = 0.493570;
		Omega[2] = 0.166220;
		Omega[3] = 0.072646;
		Omega[4] = 0.082558;
		Omega[7] = 0.056058;
		Omega[8] = 0.037229;
		Omega[18] = 0.055590;
		Omega[63] = 0.025023;
		Omega[65] = 0.003135;
#else
		Omega[0] = 0.00505375;
		Omega[1] = 0.506158;
		Omega[2] = 0.168332;
		Omega[3] = 0.071764;
		Omega[4] = 0.0819931;
		Omega[7] = 0.0546523;
		Omega[8] = 0.0352238;
		Omega[18] = 0.054178;
		Omega[63] = 0.0226281;
		Omega[65] = 0.0000173744;
#endif
		maxDegree = 66;

	}


	double summation = 0.0;
	// Summation. 做Normalization用
	for (degree = 1; degree <= MBCount; degree++){
		summation += Omega[degree - 1];
	}

	// Normalization and Cumulation. 做機率表用
	for (degree = 1; degree <= MBCount; degree++){
		Omega[degree - 1] = Omega[degree - 1] / summation;

		if (CumuFlag){
			if (degree != 1)
				Omega[degree - 1] += Omega[degree - 2];
		}
	}
	return Omega;
}

double *LayerProGenerator(double *ProbTable)
{
	double summation = 0.0;
	//double LayerCount = sizeof(*ProbTable) / sizeof(ProbTable);
	for (int i = 0; i < LayerCount; i++) {
		summation += ProbTable[i];
	}
	double *culLayerWeight = new double[(int)LayerCount];

	// Normalization and Cumulation.
	for (int i = 0; i < LayerCount; i++){
		culLayerWeight[i] = ProbTable[i] / summation;
		if (i)
			culLayerWeight[i] += ProbTable[i - 1];
	}
	return culLayerWeight;
}

void Recorder()
{
	ofstream output;
	time_t nows = time(0);
	tm *ltm = localtime(&nows);

	output.open("output.csv", ios::app | ios::in | ios::out);
	double recoveryLayer1 = ((double)recoveryIFrameBlock / (double)totalIFrameBlock) * 100.0;
	double recoveryLayer2 = ((double)recoveryRFrameBlock / (double)totalRFrameBlock) * 100.0;
	double pkylossRate = (double)(totalPKT - recvPKT) / (double)totalPKT *100.0;
	output << (double)w0 << "," << OverHead << "," << a << "," << recoveryLayer1 << "," << recoveryLayer2 << ",";
	output << recoveryIFrameBlock << "," << recoveryRFrameBlock << "," << totalIFrameBlock << "," << totalRFrameBlock << "," << recvPKT << "," << totalPKT << ",";
	output << destoryIframe << "," << destoryPframe << "," << totalIframe << "," << totalPframe<<",";
	output << 1900 + ltm->tm_year << "-" << 1 + ltm->tm_mon << "-" << ltm->tm_mday << " " << ltm->tm_hour << ":" << ltm->tm_min << ":" << ltm->tm_sec << ",";
	output << pkylossRate << "," << ChannelCodingType << endl;

	output.close();
}

void StreamPlayer()
{
	ofstream recvRecorder, destoryRecorder, seekRecorder;
	recvRecorder.open("recvRecorder.log", ios::ate);
	destoryRecorder.open("destoryRecorder.csv", ios::ate);
	seekRecorder.open("seekRecorder.log", ios::ate);
	//while (!recvStart);
	while (DM.size() < startupDMSize);
	boost::this_thread::sleep(boost::posix_time::seconds(startupdelay));
	bool firstPacket = true;
	uint8_t *data;
	uint8_t *tdata = new uint8_t[BlockSize];
	int length;
	uint64_t cnt = 0, times = 0;
	uint16_t threshold = 10000;
	bool hasRecord = false;
	FRAMEINFO FIO;
	while (true){
		FIQ.wait_and_pop(FIO);
		if (FIO.m == 0)
			break;
		cnt++;
		length = FIO.size;
		data = new uint8_t[length];
		int totalDiv = (int)ceil((double)length / BlockSize);
		int remd = FIO.size;

		if (FIO.flags&AV_PKT_FLAG_KEY){
			destoryRecorder << "K:" << FIO.size << ",";
			++totalIframe;
		}
		else{
			destoryRecorder << "R:" << FIO.size << ",";
			++totalPframe;
		}

		for (int offset = 0; remd > 0; offset++){
			int blockIndex = FIO.loc + (offset*BlockSize);
			std::memset(tdata, 0xFF, sizeof(uint8_t)*(BlockSize));
			//cout << blockIndex << endl;
			if (DM.find(blockIndex)){
				memcpy(tdata, (uint8_t*)DM.value(blockIndex), sizeof(uint8_t)*BlockSize);
				destoryRecorder << blockIndex << ",";
			}
			else{
				destoryRecorder << "-1,";
				if (!hasRecord){
					if (FIO.flags&AV_PKT_FLAG_KEY){
						++destoryIframe;
					}
					else{
						++destoryPframe;
					}
					hasRecord = true;
				}
			}
			seekRecorder << blockIndex << endl;
			if (remd<BlockSize)
				std::memcpy(data + (offset*BlockSize), tdata, sizeof(uint8_t)*remd);
			else
				std::memcpy(data + (offset*BlockSize), tdata, sizeof(uint8_t)*BlockSize);
			remd -= BlockSize;
		}

		hasRecord = false;
		destoryRecorder << endl;
		AVPacket *nwpacket = new AVPacket();
		av_init_packet(nwpacket);
		nwpacket->size = length;
		nwpacket->data = data;
		nwpacket->stream_index = FIO.stream_index;
		nwpacket->flags = FIO.flags;
		nwpacket->pts = FIO.pts;
		nwpacket->dts = FIO.dts;
		nwpacket->stream_index = 0;
		//nwpacket->flags = AV_PKT_FLAG_KEY;
		//nwpacket->pts = AV_NOPTS_VALUE;
		//nwpacket->dts = AV_NOPTS_VALUE;
		libraryOutput->push(nwpacket);

		if (firstPacket){
			AVCodecID avcid;
			avcid = AV_CODEC_ID_H264;
			boost::thread *vT = new boost::thread(&videoThread, libraryOutput, frameBufferSize, avcid);
			firstPacket = false;
			boost::this_thread::sleep(boost::posix_time::microseconds(viddelay));
		}
		else{
			boost::this_thread::sleep(boost::posix_time::microseconds(viddelay));
		}
	}
	playDone = true;
	//Recorder();
	destoryRecorder.close();
	seekRecorder.close();
}

int main(int argc, char *argv[])
{
	//int i = 2147483647;
	//cout << i << endl;

//	cout << sizeof(PACKET) << endl;
//	system("pause");
//	return 1;
	time_t ptime, ltime;
	ptime = boost::posix_time::microsec_clock::local_time().time_of_day().total_nanoseconds();
	DM.clear();
	SM.clear();
	SQ.clear();
	WQ.clear();
	DQ.clear();
	boost::property_tree::ini_parser::read_ini("transmitter.ini", config);
	libraryOutput = new tsQueue<AVPacket*>();
	playerInput = new tsQueue<AVPacket*>();
	ffmpegInit();
	//DDT = RaptorDistribution(1); // Store Initial Degree Distribution Table.
	//DDTT = RaptorDistribution(0); // Store Degree Probability Table.

	Interval = config.get<time_t>("RateControl.Interval");
	cwnd = config.get<int>("RateControl.cwnd");
	WinBSize = config.get<int>("ChannelCoding.WinBSize");
	BlockSize = config.get<int>("ChannelCoding.BlockSize");
	TrDelay = config.get<int>("RateControl.Delay");
	ChannelCodingType = config.get<int>("Other.channelCoding");
	w0 = config.get<double>("ChannelCoding.w0");
	K = config.get<int>("ChannelCoding.K");
	a = config.get<double>("ChannelCoding.a");
	OverHead = config.get<double>("ChannelCoding.e");
	framerate = config.get<double>("Other.fps");
	viddelay = static_cast<int>(T_MICROSECONDS / framerate);
	startupdelay = config.get<int>("Other.startupdelay");
	startupDMSize = config.get<int>("Other.startupDMSize");
#if EXPERIMENT
	if (argc == 1){

		///////////////////////////////////////////////
		//						做 Receiver
		///////////////////////////////////////////////

		WinBSize = K*BlockSize;
		MBCount = WinBSize / BlockSize;
		layerWeight = new double[LayerCount]{ w0, (1.0 - w0) };

		int ly1, ly2;
		ly1 = static_cast<int>(WinBSize*a);
		ly2 = WinBSize - ly1;
		LayerSize = new int[LayerCount]{ ly1, ly2 };

		lossRate = new double[2]{ loss, (1.0 - loss) };
		
		DDT = RaptorDistribution(1); // Store Initial Degree Distribution Table.
		DDTT = RaptorDistribution(0); // Store Degree Probability Table.

		std::thread workerVideoSplitter(&VideoSplitter);
		workerVideoSplitter.join();
		cout << "workerVideoSplitter Done!" << endl;
		std::thread workerWBCreator(&WBCreator);
		workerWBCreator.join();
		cout << "workerWBCreator Done!" << endl;

		std::thread workerReceiverInit(&receiverInit);
		workerReceiverInit.join();
		//cout << "workerReceiverInit Done!" << endl;

		//std::cout << "inited." << endl;
		//std::thread workerReceiverTCP(&receiverTCP);
		std::thread workerReceiver(&receiver);
		std::thread workerPKTProcess(&PKTProcess);
		
		//cout << "workerReceiverInit Start!" << endl;
		//std::thread workerReceiverTCPFeedBack(&receiverTCPFeedBack);
		//while (recvStart);

		std::thread workerDecoder(&decoder);
		//cout << "workerDecoder Start!" << endl;
		std::thread workerChecker(&checker);
		//cout << "workerChecker Start!" << endl;
		std::thread workerStreamPlayer(&StreamPlayer);
		workerStreamPlayer.join();
		//workerReceiverTCP.join();
		//workerReceiverTCPFeedBack.join();
		workerReceiver.join();
		workerPKTProcess.join();
		workerDecoder.join();
		workerChecker.join();
	}
	else{
		///////////////////////////////////////////////
		//						做 Transmitter
		///////////////////////////////////////////////

		//w0 = OverHead1 = 0.5;
		//OverHead = OverHead2 = 0.6;
		//a = 0.5;
		//loss = 0.0;
		//K = 2000;

		if (argc < 2) {
			fprintf(stderr, "usage: [-w w0] [-a alpha] [-e overhead]\n", argv[0]);
			//exit(1);
		}
		else {
			for (int i = 1; i < argc; ++i) {
				if (argv[i][0] == '-') {
					switch (argv[i][1]) {
					case 'w':
						if (argc > i + 1) w0 = OverHead1 = atof(argv[++i]); break;
					case 'a':
						if (argc > i + 1) a = atof(argv[++i]); break;
					case 'c':
						if (argc > i + 1) parameter_C = atof(argv[++i]); break;
					case 'd':
						if (argc > i + 1){ delta = atof(argv[++i]); RSDSwitch = true; }
						break;
					case 'e':
						if (argc > i + 1) OverHead = OverHead2 = atof(argv[++i]); break;
					case 'l':
						if (argc > i + 1) loss = atof(argv[++i]); break;
					case 's':
						if (argc > i + 1) encodesleep = atoi(argv[++i]); break;
					case 'k':
						if (argc > i + 1) K = atoi(argv[++i]); break;
					default: break;
					}
				}
			}
		}

		WinBSize = K*BlockSize;
		MBCount = WinBSize / BlockSize;
		layerWeight = new double[LayerCount]{ w0, (1.0 - w0) };
		int ly1, ly2;
		ly1 = static_cast<int>(WinBSize*a);
		ly2 = WinBSize - ly1;
		LayerSize = new int[LayerCount]{ ly1, ly2 };

		lossRate = new double[2]{ loss, (1.0 - loss) };
		DDT = RaptorDistribution(1); // Store Initial Degree Distribution Table.
		DDTT = RaptorDistribution(0); // Store Degree Probability Table.

		std::thread workerTransmitterInit(&transmitterInit);
		workerTransmitterInit.join();
		std::thread workerVideoSplitter(&VideoSplitter);
		workerVideoSplitter.join();
		std::thread workerWBCreator(&WBCreator);
		workerWBCreator.join();


		//while (!splitDone);
		std::thread workerEncoder(&encoder);
		std::thread workerNonEncoder(&nonencoder);
	

		//while (!encoDone);
		//std::thread workerTransmitterTCP(&transmitterTCP);
		std::thread workerTransmitter(&transmitter);
		//std::thread workerTransmitterTCPReceptionist(&transmitterTCPReceptionist);
		std::thread workerTransmitterReceptionist(&TransmitterReceptionist);

		workerEncoder.join();
		workerNonEncoder.join();

		//workerTransmitterTCP.join();
		workerTransmitter.join();
		//workerTransmitterTCPReceptionist.join();
		workerTransmitterReceptionist.join();
	}

#else
	//w0 = OverHead1 = 0.50f;
	//OverHead = OverHead2 = 0.61f;

	//a = 0.50f;
	loss = 0.0;
	if (argc < 2) {
		fprintf(stderr, "usage: [-w w0] [-a alpha] [-e overhead]\n", argv[0]);
		//exit(1);
	}
	else {
		for (int i = 1; i < argc; ++i) {
			if (argv[i][0] == '-') {
				switch (argv[i][1]) {
				case 'w': if (argc > i + 1) w0 = OverHead1 = atof(argv[++i]); break;
				case 'a': if (argc > i + 1) a = atof(argv[++i]); break;
				case 'c': if (argc > i + 1) parameter_C = atof(argv[++i]); break;
				case 'd': if (argc > i + 1){ delta = atof(argv[++i]); RSDSwitch = true; } break;
				case 'e': if (argc > i + 1) OverHead = OverHead2 = atof(argv[++i]); break;
				case 'l': if (argc > i + 1) loss = atof(argv[++i]); break;
				default: break;
				}
			}
		}
	}


	WinBSize = static_cast<int>(K*BlockSize);
	int ly1, ly2;
	ly1 = static_cast<int>(WinBSize*a);
	ly2 = WinBSize - ly1;
	
	MBCount = K;
	layerWeight = new double[LayerCount]{ w0, (1.0 - w0) };
	LayerSize = new int[LayerCount]{ ly1, ly2 };
	lossRate = new double[2]{ (1.0 - loss), loss };

	DDT = RaptorDistribution(1); // Store Initial Degree Distribution Table.
	DDTT = RaptorDistribution(0); // Store Degree Probability Table.


	std::thread workerVideoSplitter(&VideoSplitter);
	workerVideoSplitter.join();
	std::thread workerWBCreator(&WBCreator);
	workerWBCreator.join();
	//std::thread workerEncoder(&encoder);
	//workerEncoder.join();
#if 1
	std::thread workerEncoder(&encoder);
	std::thread workerNonEncoder(&nonencoder);
	std::thread workerReceiver(&receiver);
	std::thread workerDecoder(&decoder);
	std::thread workerChecker(&checker);
	//boost::thread workerStreamPlayer(&StreamPlayer);
	//workerStreamPlayer.join();
	workerEncoder.join();
	workerNonEncoder.join();
	workerReceiver.join();
	workerDecoder.join();
	workerChecker.join();
#endif
	cout << "Done" << endl;


#endif
	ltime = boost::posix_time::microsec_clock::local_time().time_of_day().total_nanoseconds();
	std::cout << "Elapsed time:" << (ltime - ptime) / 1000000000 << "s" << endl;
	std::system("pause");
	return 0;
}
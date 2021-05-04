
#ifndef OMNICARTRANSCEIVER_H
#define OMNICARTRANSCEIVER_H

#include <iostream>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

#include <mutex>

#define SERVER_IP "127.0.0.1"
#define TCP_SERVER_PORT 10001
#define UDP_SERVER_PORT 10002
#define BUFFER_SIZE 4096
#define TIMEOUT 10

namespace ORB_SLAM2
{

class FrameDrawer;
class System;

class OmniCarTransceiver
{
public:
    OmniCarTransceiver(System* pSystem, FrameDrawer* pFrameDrawer, const string &strSettingPath);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

private:

    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

    void udp_tx();
    void tcp_rx();
};

}

#endif // OMNICARTRANSCEIVER_H
	


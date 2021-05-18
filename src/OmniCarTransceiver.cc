#include "OmniCarTransceiver.h"
#include <pangolin/pangolin.h>

#include <mutex>

#include <unistd.h> // fixes "usleep" error

namespace ORB_SLAM2
{

OmniCarTransceiver::OmniCarTransceiver(System* pSystem, FrameDrawer *pFrameDrawer, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

void OmniCarTransceiver::Run()
{
    mbFinished = false;
    mbStopped = false;

    bool bFollow = true;
    bool bLocalizationMode = false;

    thread tcp_thread(&OmniCarTransceiver::tcp_rx, this);
    thread udp_thread(&OmniCarTransceiver::udp_tx, this);
    thread serial_thread(&OmniCarTransceiver::serial_talk, this);
    cout << "TCP and UDP threads were started" << endl;

    while(1)
    {
        // if(bFollow)
        // {
        //     bFollow = false;
        // }

        // if(!bLocalizationMode)
        // {
        //     mpSystem->ActivateLocalizationMode();
        //     bLocalizationMode = true;
        // }
        // else if(bLocalizationMode)
        // {
        //     mpSystem->DeactivateLocalizationMode();
        //     bLocalizationMode = false;
        // }

        // cv::Mat im = mpFrameDrawer->DrawFrame();
        // cv::imshow("ORB-SLAM2: Current Frame",im);
        // cv::waitKey(mT);

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
    tcp_thread.join();
    udp_thread.join();
    serial_thread.join();
}

void OmniCarTransceiver::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool OmniCarTransceiver::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void OmniCarTransceiver::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool OmniCarTransceiver::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void OmniCarTransceiver::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool OmniCarTransceiver::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool OmniCarTransceiver::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void OmniCarTransceiver::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void OmniCarTransceiver::udp_tx()
{
    // Create a socket
    int serverSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (serverSocket == -1)
    {
        cerr << "Error: Can't create a socket" << endl;
    }

    // Bind the ip address and port to a socket
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(UDP_SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &serverAddress.sin_addr);

    bind(serverSocket, (sockaddr *)&serverAddress, sizeof(serverAddress));

    // Get init message from client
    char buffer[BUFFER_SIZE];
    memset(buffer, 0, BUFFER_SIZE);
    sockaddr_in clientAddress;
    socklen_t clientSize = sizeof(clientAddress);
    int bytesReceived = recvfrom(serverSocket, buffer, BUFFER_SIZE, 0, (sockaddr *)&clientAddress, &clientSize);
    if (bytesReceived == -1)
    {
        cerr << "Error in recvfrom()" << endl;
    }
    cout << "Received init message -> starting video transmission" << endl;

    cv::Mat img;
    while (true)
    {
        img = mpFrameDrawer->DrawFrame();
        if (img.empty())
            break;

        vector<uchar> imgBuffer;
        std::vector<int> param(2);
        param[0] = cv::IMWRITE_JPEG_QUALITY;
        param[1] = 50; //default(95) 0-100
        imencode(".jpg", img, imgBuffer, param);
        int imgBufferSize = imgBuffer.size();
        cout << "Encoded image size: " << imgBufferSize << endl;

        int bytesSent = sendto(serverSocket, &imgBuffer[0], imgBufferSize, 0, (sockaddr *)&clientAddress, clientSize);
        if (bytesSent == -1)
        {
            cout << "Error sending image" << endl;
        }
        // imshow("TRANSMITTING", img);
        if (cv::waitKey(TIMEOUT) >= 0)
            break;
    }
 
    // Close the socket
    close(serverSocket);
}

void OmniCarTransceiver::tcp_rx()
{
    // Create a socket
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1)
    {
        cerr << "Error: Can't create a socket" << endl;
    }

    // Bind the ip address and port to a socket
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(TCP_SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &serverAddress.sin_addr);

    bind(serverSocket, (sockaddr *)&serverAddress, sizeof(serverAddress));

    // Tell the socket is for listening
    listen(serverSocket, SOMAXCONN);

    // Wait for a connection
    sockaddr_in clientAddress;
    socklen_t clientSize = sizeof(clientAddress);
    int clientSocket = accept(serverSocket, (sockaddr *)&clientAddress, &clientSize);

    char hostName[NI_MAXHOST];   // Client's remote name
    char clientPort[NI_MAXSERV]; // Service (i.e. port) the client is connect on

    memset(hostName, 0, NI_MAXHOST);
    memset(clientPort, 0, NI_MAXSERV);

    if (getnameinfo((sockaddr *)&clientAddress, sizeof(clientAddress), hostName, NI_MAXHOST, clientPort, NI_MAXSERV, 0) == 0)
    {
        cout << hostName << " connected on port " << clientPort << endl;
    }
    else
    {
        inet_ntop(AF_INET, &clientAddress.sin_addr, hostName, NI_MAXHOST);
        cout << hostName << " connected on port " << ntohs(clientAddress.sin_port) << endl;
    }

    // Close listening (server) socket
    close(serverSocket);

    // While loop: accept and echo message back to client
    char buffer[BUFFER_SIZE];
    while (true)
    {
        // Wait for client to send data
        memset(buffer, 0, BUFFER_SIZE);
        int bytesReceived = recv(clientSocket, buffer, BUFFER_SIZE, 0);
        if (bytesReceived == -1)
        {
            cerr << "Error in recv()" << endl;
            break;
        }

        if (bytesReceived == 0)
        {
            cout << "Client disconnected " << endl;
            break;
        }
        // cout << "Received " << bytesReceived << "bytes" << endl;

        if (bytesReceived == 3)
        {
            unique_lock<mutex> lock(mutexControl);
            memcpy(controlVec, buffer, 3);
            // printf("Got [%i, %i, %i] from TCP\n", controlVec[0], controlVec[1], controlVec[2]);
        }
    }

    // Close client socket
    close(clientSocket);
    unique_lock<mutex> lock(mutexControl);
    controlVec[0] = 127;
    controlVec[1] = 127;
    controlVec[2] = 127;
}

void OmniCarTransceiver::serial_talk()
{
    const char *portName = "/dev/ttyACM0";
    float feedbackPos[3] = {0, 0, 0};
    char n = '\n';
    int bytesSent = 0;
    int bytesRead = 0;

    const int bufferSize = 14;
    char buffer[bufferSize] = {0};
    uint8_t readChecksum;
    uint8_t calcChecksum;

    int fd = openPort(portName);
    configPort(fd);
    srand(static_cast<unsigned>(time(0)));

    while (1)
    {
        //tx
        {
            unique_lock<mutex> lock(mutexControl);
            calcChecksum = crc8((uint8_t *)controlVec, 3);
            bytesSent = 0;
            bytesSent += write(fd, controlVec, 3);
            bytesSent += write(fd, &calcChecksum, 1);
            bytesSent += write(fd, &n, 1);
            // printf("Just sent %i bytes to Arduino\n", bytesSent);
        }

        //rx
        for (int i = 0; i < 14; i++)
        {
            bytesRead = read(fd, buffer + i, 1);
        }

        if (buffer[13] == '\n')
        {
            readChecksum = buffer[0];
            memcpy(feedbackPos, buffer + 1, 12);
            calcChecksum = crc8((uint8_t *)feedbackPos, 12);
            bool isPassed = readChecksum == calcChecksum;
            if (isPassed)
            {
                // printf("Received accurate: [%f; %f, %f];\n\n", feedbackPos[0], feedbackPos[1], feedbackPos[2]);
            }
        }
    }
}
}

#include <iostream>
#include <opencv2/core/core.hpp>
#include <System.h>
#include <chrono>

using namespace std;

int main() {
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM("../Vocabulary/ORBvoc.bin", "../WebCam.yaml", ORB_SLAM2::System::MONOCULAR, false);

    // Capture frames from webcam
    cv::Mat frame;
    cv::VideoCapture cap;
    cap.open(0, cv::CAP_ANY);
    if (!cap.isOpened())
    {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    cout << "Start grabbing" << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    while (1)
    {
        cap.read(frame);
        if (frame.empty())
        {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        t2 = chrono::steady_clock::now();
        double interval = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        SLAM.TrackMonocular(frame, interval);
        t1 = chrono::steady_clock::now();
        // cv::imshow("Live", frame);

        // if (cv::waitKey(5) >= 0)
        //     break;
    }

    SLAM.Shutdown();
    cout << "Finished by Divelix" << endl;
    return 0;
}
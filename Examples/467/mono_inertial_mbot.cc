/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*
*
* Adapted for use in umich EECS 467 assignment A3
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <queue>
#include <thread>
#include <mutex>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/mbot_imu_t.hpp"
#include "lcmtypes/mbot_video_stream_t.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <System.h>
#include "ImuTypes.h"

#define MBOT_IMU_CHANNEL "MBOT_IMU"
#define MBOT_VIDEO_CHANNEL "MBOT_VIDEO_STREAM"

using namespace std;

void mbot_imu_handler(const lcm_recv_buf_t *rbuf, const char *channel,
    const mbot_imu_t *msg, void *user);
void mbot_video_stream_handler(const lcm_recv_buf_t *rbuf, const char *channel,
    const mbot_video_stream_t *msg, void *user);
void sync_img_imu();

double ttrack_tot = 0;


ORB_SLAM3::System* mpSLAM;
bool mbClahe; //set to false to fix compile errors
cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

queue< vector<float> > imuBuf;
queue<double> imuBufTimes;
std::mutex imuMtx;

queue<double> imgBufTimes;
queue<cv::Mat> imgBuf;
std::mutex imgMtx;


/*
struct mbot_imu_t
{
    int64_t utime;

    float gyro[3];
    float accel[3];
    float mag[3];
    float tb_angles[3];
    float temp;
}
*/

class Handler
{
public:
    ~Handler() {}

    void mbot_video_stream_handler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const mbot_video_stream_t* msg)
    {
        imgMtx.lock();
        // Use nanoseconds since epoch for consistency
        auto tImg = std::chrono::steady_clock::now();
        imgBufTimes.push(std::chrono::duration_cast<std::chrono::nanoseconds>(tImg.time_since_epoch()).count()/1e9); 

        cv::Mat img_recv = imdecode(msg->image, IMREAD_GRAYSCALE);
        imgBuf.push(img_recv);
        imgMtx.unlock();
    }

    void mbot_imu_handler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const mbot_imu_t* msg)
    {
        // int64_t utime; float gyro[3]; float accel[3]; float mag[3]; float tb_angles[3]; float temp;
        imuMtx.lock();
        // Use nanoseconds since epoch for consistency ; not using msg->utime
        auto tImu = std::chrono::steady_clock::now();
        imuBufTimes.push(std::chrono::duration_cast<std::chrono::nanoseconds>(tImu.time_since_epoch()).count()/1e9); 
        
        vector<float> imu;
        imu.resize(13);
        for(short i = 0; i < 3; ++i){
            imu[0 + i] = (msg->gyro[i]);
            imu[3 + i] = (msg->accel[i]);
            imu[6 + i] = (msg->mag[i]);
            imu[9 + i] = (msg->tb_angles[i]);
        }
        imu[12] = msg->temp;
        imuBuf.push(imu);

        imuMtx.unlock();
    }

};
    

int main(int argc, char *argv[])
{

    if(argc < 3)
    {
        cerr << endl << "Usage: ./mono_inertial_euroc path_to_vocabulary path_to_settings" << endl;
        return 1;
    }
    if(argc >= 4){
        std::string sbEqual(argv[3]);
        if(sbEqual == "true")
        {
            mbClahe = true;
        }
    }
    else{
        mbClahe = false;
    }

    lcm::LCM lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");
    Handler HandlerObject;
    lcm.subscribe(MBOT_IMU_CHANNEL, 
    			  &Handler::mbot_imu_handler, 
    			  &HandlerObject);

    lcm.subscribe(MBOT_VIDEO_CHANNEL, 
    			&Handler::mbot_video_stream_handler, 
    			&HandlerObject);   
    

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    // vTimesTrack.resize(tot_images); cannot resize bc unknown img stream length due to realtime

    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true);

    // Set global instance to point to above
    mpSLAM = &SLAM;

    // Main loop
    //cv::Mat im;
    std::thread sync_thread(&sync_img_imu);
    while(1)
    {
        
        char key = (char) cv::waitKey(1);
        if(key == 113 || key == 27) break; // 113 is 'q' key, 27 is 'ESC' key
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    return 0;
}

void sync_img_imu(){
    while(1)
    {
        cv::Mat img;
        double tImg;
        // Wait until we have image and imu data to process
        if(!imgBuf.empty() && !imuBuf.empty()){
            tImg = imgBufTimes.front();
            if(tImg > imuBufTimes.back())
                continue;
            
            imgMtx.lock();
            img = imgBuf.front();
            imgBuf.pop();
            imgBufTimes.pop();
            imgMtx.unlock();            
        

            // Load imu measurements
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            imuMtx.lock();
            if(!imuBuf.empty())
            {
                cout << "t_cam " << tImg << endl; // good lil debug line for when it doesnt work on first try
                                                  // number of times printed corresponds to num imu steps between frames

                vImuMeas.clear();
                while(!imuBufTimes.empty() && imuBufTimes.front()<=tImg)
                {
                    /*vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                                vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                                vTimestampsImu[seq][first_imu[seq]]));*/
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(imuBuf.front()[3], imuBuf.front()[4], imuBuf.front()[5],
                                                            imuBuf.front()[0], imuBuf.front()[1], imuBuf.front()[2],
                                                            imuBufTimes.front()));
                    imuBufTimes.pop();
                    imuBuf.pop();
                }
            }
            imuMtx.unlock();
            if(mbClahe)
            {
                mClahe->apply(img,img);
            }
            
            
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
            mpSLAM->TrackMonocular(img,tImg,vImuMeas);

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            ttrack_tot += ttrack;
            std::cout << "ttrack: " << ttrack << std::endl;
        }
        

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

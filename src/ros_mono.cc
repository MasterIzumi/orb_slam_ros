/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 2)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    string voc_dir = "/home/zl/catkin_ws/src/parkingEnvSensing/Vocabulary/ORBvoc.txt";
    string config_dir = argv[1];
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(voc_dir,config_dir,ORB_SLAM2::System::MONOCULAR,false);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double timread= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    cv::Mat cameraPose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
    double tall= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
    
    cout << "Image reading time = " << timread << "s, freqency = " << 1/timread << "Hz" << endl;
    cout << "Tracking time = " << ttrack << "s, freqency = " << 1/ttrack << "Hz" << endl; 
    cout << "ALL cost time = " << tall << "s, freqency = " << 1/tall << "Hz\n" << endl; 
    
}



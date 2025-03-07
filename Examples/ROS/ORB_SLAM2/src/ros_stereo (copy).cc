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

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify = false;
    cv::Mat M1l,M2l,M1r,M2r;
};

void GetPointCloud()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    ros::NodeHandle nh;
    //ros::Publisher orbpoint2_pub = nh.advertise<sensor_msgs::PointCloud2>("orb_point2", 1);
    ros::Publisher orbMappoint2_pub = nh.advertise<sensor_msgs::PointCloud2>("orb_Mappoint2", 1);
    //pcl::PointCloud<pcl::PointXYZ> orb_cloud2;
    pcl::PointCloud<pcl::PointXYZ> orbMap_cloud2; //MapPoints中的点
    //sensor_msgs::PointCloud2 orb_toRoscloud2;
    sensor_msgs::PointCloud2 orbMap_toRoscloud2;

    if(argc != 1)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    
    std::string vocstr="/home/shinan/Downloads/ORB_SLAM2-master/Vocabulary/ORBvoc.txt";
    std::string yamlstr = "/home/shinan/Downloads/ORB_SLAM2-master/Examples/Stereo/EuRoC.yaml";

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocstr.data(), yamlstr.data(), ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    //stringstream ss(argv[3]);
    //ss >> boolalpha >> igb.do_rectify;
    igb.do_rectify =false; //shinan:set to false to not rectify
    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    /*ros::NodeHandle nh;

   essage_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
   message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "camera/right/image_raw", 1);
   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
   message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
   sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));*/
    int num=0;
    while(ros::ok()){
        cv::VideoCapture cap1(0);
        cv::VideoCapture cap2(2);
        cv::Mat src1_img,src2_img;
        double tframe = 20;
        while(cap1.read(src1_img) && cap2.read(src2_img))
        {
            cv::Mat mTcw = igb.mpSLAM->TrackStereo(src1_img,src2_img,tframe);

            //获取当前track下的地图点
            auto mvpMapPoints = igb.mpSLAM->GetTrackedMapPoints();

            int valibleMapPointNum = 0;
            for (auto mappoint : mvpMapPoints) {
                if (mappoint != NULL) {
                    valibleMapPointNum++;
                }
            }

            orbMap_cloud2.width = valibleMapPointNum;
            orbMap_cloud2.height = 1;
            orbMap_cloud2.points.resize(orbMap_cloud2.width * orbMap_cloud2.height);
            if (!mTcw.empty()) {
                for (auto points : mvpMapPoints) {
                    if(points==NULL)continue;

                    cv::Mat WorldPos = cv::Mat_<float>(4, 1);
                    points->GetWorldPos().col(0).copyTo(WorldPos.rowRange(0,3).col(0));
                    WorldPos.at<float>(3) = 1;
                    // std::cout << "WorldPos" << WorldPos << std::endl;
                    cv::Mat poi = mTcw * WorldPos;
                    //std::cout << "poi" << poi << std::endl;
                    orbMap_cloud2.points[num].x = poi.at<float>(2);
                    orbMap_cloud2.points[num].y = -poi.at<float>(0);
                    orbMap_cloud2.points[num].z = -poi.at<float>(1);
                    num++;
                }
            }
            pcl::toROSMsg(orbMap_cloud2, orbMap_toRoscloud2);
            //pcl::toROSMsg(orb_cloud2, orb_toRoscloud2);
            orbMap_toRoscloud2.header.frame_id = "ORB_frame";
            //orb_toRoscloud2.header.frame_id = "ORB_frame";
            orbMap_toRoscloud2.header.stamp = ros::Time::now();
            //orb_toRoscloud2.header.stamp = ros::Time::now();
            orbMappoint2_pub.publish(orbMap_toRoscloud2);
            //orbpoint2_pub.publish(orb_toRoscloud2);
        }



    }



    //ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

}



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

//#include<ros/ros.h>
//#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include"System.h"

#include"ViewerAR.h"

using namespace std;


ViewerAR viewerAR;
bool bRGB = true;

cv::Mat K;
cv::Mat DistCoef;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(/*const sensor_msgs::ImageConstPtr& msg*/);

    ORB_SLAM2::System* mpSLAM;
};

string HOST="D:\\";
string VOC_PATH=HOST+"Projects/SLAM/ORB_SLAM2-master/Vocabulary/ORBvoc.txt";
string SETTING_PATH=HOST+"Projects/SLAM/ORB_SLAM2-master/Examples/Monocular/TUM1.yaml";
int camIndex,previewWidth,previewHeight;
void init()
{
    ifstream in;
    in.open("config.txt");
    getline(in,HOST);

    getline(in,VOC_PATH);
    VOC_PATH=HOST+VOC_PATH;

    getline(in,SETTING_PATH);
    SETTING_PATH=HOST+SETTING_PATH;

    in>>camIndex>>previewWidth>>previewHeight;
    in.close();
}
cv::VideoCapture cap;
double timeStamp;
int main()
{
    init();
//    ros::init(argc, argv, "Mono");
//    ros::start();

//    if(argc != 3)
//    {
//        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
//        ros::shutdown();
//        return 1;
//    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(VOC_PATH,SETTING_PATH,ORB_SLAM2::System::MONOCULAR);


    cout << endl << endl;
    cout << "-----------------------" << endl;
    cout << "Augmented Reality Demo" << endl;
    cout << "1) Translate the camera to initialize SLAM." << endl;
    cout << "2) Look at a planar region and translate the camera." << endl;
    cout << "3) Press Insert Cube to place a virtual cube in the plane. " << endl;
    cout << endl;
    cout << "You can place several cubes in different planes." << endl;
    cout << "-----------------------" << endl;
    cout << endl;


    viewerAR.SetSLAM(&SLAM);

    cap.open(camIndex);
    if(!cap.isOpened()) cap.open(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,previewWidth);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,previewHeight);
    //cap.set(CV_CAP_PROP_FPS,30);
    timeStamp=0;
    ImageGrabber igb(&SLAM);

//    ros::NodeHandle nodeHandler;
//    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);


    cv::FileStorage fSettings(SETTING_PATH, cv::FileStorage::READ);
    bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
    float fps = fSettings["Camera.fps"];
    viewerAR.SetFPS(fps);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    viewerAR.SetCameraCalibration(fx,fy,cx,cy);

    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    DistCoef = cv::Mat::zeros(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

    thread tViewer = thread(&ViewerAR::Run,&viewerAR);

    while(1) igb.GrabImage();
    //ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    //ros::shutdown();

    return 0;
}


void ImageGrabber::GrabImage(/*const sensor_msgs::ImageConstPtr& msg*/)
{
    // Copy the ros image message to cv::Mat.
//    cv_bridge::CvImageConstPtr cv_ptr;
//    try
//    {
//        cv_ptr = cv_bridge::toCvShare(msg);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
    cv::Mat im;//cv_ptr->image.clone();
    cap.read(im);
    cv::Mat imu;
    cv::Mat Tcw= mpSLAM->TrackMonocular(im,timeStamp+1/60.0);
    int state = mpSLAM->GetTrackingState();
    vector<ORB_SLAM2::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
    vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();

    cv::undistort(im,imu,K,DistCoef);

    bRGB=false;
    if(bRGB)
        viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);
    else
    {
        cv::cvtColor(imu,imu,CV_RGB2BGR);
        viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);
    }
}

//#include <GLFW/glfw3.h>
//
//int main(void)
//{
//    GLFWwindow* window;
//
//    /* Initialize the library */
//    if (!glfwInit())
//        return -1;
//
//    /* Create a windowed mode window and its OpenGL context */
//    window = glfwCreateWindow(480, 320, "Hello World", NULL, NULL);
//    if (!window)
//    {
//        glfwTerminate();
//        return -1;
//    }
//
//    /* Make the window's context current */
//    glfwMakeContextCurrent(window);
//
//    /* Loop until the user closes the window */
//    while (!glfwWindowShouldClose(window))
//    {
//        /* Draw a triangle */
//        glBegin(GL_TRIANGLES);
//
//        glColor3f(1.0, 0.0, 0.0);    // Red
//        glVertex3f(0.0, 1.0, 0.0);
//
//        glColor3f(0.0, 1.0, 0.0);    // Green
//        glVertex3f(-1.0, -1.0, 0.0);
//
//        glColor3f(0.0, 0.0, 1.0);    // Blue
//        glVertex3f(1.0, -1.0, 0.0);
//
//        glEnd();
//
//        /* Swap front and back buffers */
//        glfwSwapBuffers(window);
//
//        /* Poll for and process events */
//        glfwPollEvents();
//    }
//
//    glfwTerminate();
//    return 0;
//}
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


#ifndef VIEWERAR_H
#define VIEWERAR_H

#include <mutex>
#include <opencv2/core/core.hpp>
#include <string>
#include"../../../include/System.h"
#include <GL/glut.h>
#include <GL/freeglut.h>
class Plane
{
public:
    Plane(const std::vector<ORB_SLAM2::MapPoint*> &vMPs, const cv::Mat &Tcw);
    Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz);

    void Recompute();

    //normal
    cv::Mat n;
    //origin
    cv::Mat o;
    //arbitrary orientation along normal
    float rang;
    //transformation from world to the plane
    cv::Mat Tpw;
    float glTpw[16];
    //MapPoints that define the plane
    std::vector<ORB_SLAM2::MapPoint*> mvMPs;
    //camera pose when the plane was first observed (to compute normal direction)
    cv::Mat mTcw, XC;
};

class ViewerAR
{
public:
    ViewerAR();

    void SetFPS(const float fps){
        mFPS = fps;
        mT=1e3/fps;
    }

    void SetSLAM(ORB_SLAM2::System* pSystem){
        mpSystem = pSystem;
    }

    // Main thread function. 
    void Run();

    void SetCameraCalibration(const float &fx_, const float &fy_, const float &cx_, const float &cy_){
        fx = fx_; fy = fy_; cx = cx_; cy = cy_;
    }

    void SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status,
                      const std::vector<cv::KeyPoint> &vKeys, const std::vector<ORB_SLAM2::MapPoint*> &vMPs);

    void GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status,
                      std::vector<cv::KeyPoint> &vKeys,  std::vector<ORB_SLAM2::MapPoint*> &vMPs);

private:

    //SLAM
    ORB_SLAM2::System* mpSystem;

    void PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im);
    void AddTextToImage(const std::string &s, cv::Mat &im, const int r=0, const int g=0, const int b=0);
    void LoadCameraPose(const cv::Mat &Tcw);
    void DrawImageTexture(cv::Mat &im);
    void DrawCube(const float &size, const float x=0, const float y=0, const float z=0);
    void DrawPlane(int ndivs, float ndivsize);
    void DrawPlane(Plane* pPlane, int ndivs, float ndivsize);
    void DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<ORB_SLAM2::MapPoint*> &vMPs, cv::Mat &im);

    Plane* DetectPlane(const cv::Mat Tcw, const std::vector<ORB_SLAM2::MapPoint*> &vMPs, const int iterations=50);

    // frame rate
    float mFPS, mT;
    float fx,fy,cx,cy;

    // Last processed image and computed pose by the SLAM
    std::mutex mMutexPoseImage;
    cv::Mat mTcw;
    cv::Mat mImage;
    int mStatus;
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<ORB_SLAM2::MapPoint*> mvMPs;

    bool drawPoints;

    float camPose[16];
    GLint internal_format;
    GLuint tid;
    GLint width;
    GLint height;
    float P[16];

    void glDrawColouredCube(GLfloat axis_min=-0.5f, GLfloat axis_max = +0.5f);


    void reinitialise(GLint w, GLint h, GLint int_format, bool sampling_linear, int border, GLenum glformat, GLenum gltype,
                      void *data);

    void initGLMat(GLint width, GLint height, GLint internal_format = GL_RGB, bool sampling_linear = true, int border = 0, GLenum glformat = GL_RGB, GLenum gltype = GL_UNSIGNED_BYTE, GLvoid* data = NULL  );

    void upload(const uchar *data, GLenum data_format, GLenum data_type);

    void initViewMatrix(int w, int h, double fu, double fv, double u0, double v0, double zNear, double zFar);

};


#endif // VIEWERAR_H
	


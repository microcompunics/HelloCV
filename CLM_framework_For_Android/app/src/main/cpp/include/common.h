// Precompiled headers stuff

#ifndef __STDAFX_h_
#define __STDAFX_h_

// OpenCV stuff
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
/*#include "opencv2/objdetect.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>*/

#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include <bits/stdc++.h>
using namespace std;
// IplImage stuff (get rid of it? TODO)
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>

// C++ stuff
#include <stdio.h>

#include <fstream>
#include <iostream>
#include <sstream>

#include <vector>
#include <map>

#define _USE_MATH_DEFINES
#include <cmath>

// dlib stuff
// Used for face detection
//#include "dlib/image_processing/frontal_face_detector.h"
//#include "dlib/opencv.h"

// Boost stuff
//#include <filesystem.hpp>
//#include <filesystem/fstream.hpp>
#include <android/log.h>
#define  LOG_TAG    "JNI_PART"
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG, __VA_ARGS__)
#define LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG, __VA_ARGS__)
#define LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG, __VA_ARGS__)
#define LOGF(...)  __android_log_print(ANDROID_LOG_FATAL,LOG_TAG, __VA_ARGS__)

std::string getModelPath();
std::string getParentPath(std::string path);
void setModelPath(std::string path);
long getCurrentTime();
#endif

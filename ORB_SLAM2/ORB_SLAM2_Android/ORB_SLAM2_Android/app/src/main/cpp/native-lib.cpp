#include <jni.h>
#include <string>
#include <opencv2/opencv.hpp>
#include "include/System.h"
#include "Common.h"

#include "ORBVocabulary.h"

extern "C" {

std::string modelPath;

ORB_SLAM2::System *slamSys;
//ORB_SLAM2::ORBVocabulary* mpVocabulary;
double timeStamp;

long getCurrentTime() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}


void addTextToImage(const string &s, cv::Mat &im, const int r, const int g, const int b)
{
    int l = 10;
    cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(r,g,b),2,8);
}

void printStatus(const int &status, cv::Mat &im)
{
    switch(status) {
        case 1:  {addTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
        case 2:  {addTextToImage("SLAM ON",im,0,255,0); break;}
        case 3:  {addTextToImage("SLAM LOST",im,255,0,0); break;}
    }
}
void drawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<ORB_SLAM2::MapPoint *> &vMPs, cv::Mat &im)
{
    const int N = vKeys.size();
    for(int i=0; i<N; i++) {
        if(vMPs[i]) {
            cv::circle(im,vKeys[i].pt,2,cv::Scalar(0,255,255),-1);
        }
    }
}
void processImage(cv::Mat &image,cv::Mat &outputImage){
    timeStamp+=1/30.0;
    cv::Mat Tcw= slamSys->TrackMonocular(image,timeStamp);
    int status = slamSys->GetTrackingState();
    vector<ORB_SLAM2::MapPoint*> vMPs = slamSys->GetTrackedMapPoints();
    vector<cv::KeyPoint> vKeys = slamSys->GetTrackedKeyPointsUn();
    printStatus(status,outputImage);
    drawTrackedPoints(vKeys,vMPs,outputImage);
}

JNIEXPORT void JNICALL
Java_com_martin_ads_orb_1slam2_1android_MainActivity_nativeProcessFrame(JNIEnv *env,
                                                                        jobject instance,
                                                                        jlong matAddrGr,
                                                                        jlong matAddrRgba) {
    cv::Mat &mGr = *(cv::Mat *) matAddrGr;
    cv::Mat &mRgba = *(cv::Mat *) matAddrRgba;
    processImage(mGr,mRgba);
}

JNIEXPORT void JNICALL
Java_com_martin_ads_orb_1slam2_1android_MainActivity_initSLAM(JNIEnv *env, jobject instance,
                                                               jstring path_) {
    const char *path = env->GetStringUTFChars(path_, 0);

    modelPath=path;
    env->ReleaseStringUTFChars(path_, path);
    ifstream in;
    in.open(modelPath+"config.txt");

    string vocName,settingName;
    getline(in,vocName);
    getline(in,settingName);
    vocName=modelPath+vocName;
    //vocName="/storage/emulated/0/SLAM/ORBvoc.txt";
    settingName=modelPath+settingName;
    //settingName="/storage/emulated/0/SLAM/CameraSettings.yaml";
    timeStamp=0;
    LOGD("%s %c %s %c",vocName.c_str(),vocName[vocName.length()-1],settingName.c_str(),settingName[settingName.length()-1]);
    slamSys=new ORB_SLAM2::System(vocName,settingName,ORB_SLAM2::System::MONOCULAR);

    //mpVocabulary = new ORB_SLAM2::ORBVocabulary();
    //vocName+=".arm.bin";
    //mpVocabulary->loadFromBinFile(vocName+".arm.bin");
    //if(mpVocabulary->loadFromTextFile(vocName))
    //    LOGD("Vocabulary loaded!");
    //LOGD("Vocabulary loaded!");
}

}




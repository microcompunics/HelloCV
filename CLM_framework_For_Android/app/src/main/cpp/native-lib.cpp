#include <jni.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <bits/stdc++.h>
#include "include/CLM_core.h"

using namespace cv;
using namespace std;


extern "C" {

CLMTracker::CLMParameters *clm_parameters=NULL;
CLMTracker::CLM *clm_model=NULL;

float fx = 0, fy = 0, cx = 0, cy = 0;

double fps_vid_in = -1.0;

int frame_count = 0;

// Use for timestamping if using a webcam
int64 t_initial;

// Timestamp in seconds of current processing
double time_stamp = 0;

// Some globals for tracking timing information for visualisation
double fps_tracker = -1.0;
int64 t0 = 0;

// Visualising the results
void visualise_tracking(Mat& captured_image, Mat_<float>& depth_image, const CLMTracker::CLM& clm_model, const CLMTracker::CLMParameters& clm_parameters, int frame_count, double fx, double fy, double cx, double cy)
{

    // Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
    double detection_certainty = clm_model.detection_certainty;
    bool detection_success = clm_model.detection_success;

    double visualisation_boundary = 0.2;

    // Only draw if the reliability is reasonable, the value is slightly ad-hoc
    if (detection_certainty < visualisation_boundary)
    {
        CLMTracker::Draw(captured_image, clm_model);
        double vis_certainty = detection_certainty;
        if (vis_certainty > 1)
            vis_certainty = 1;
        if (vis_certainty < -1)
            vis_certainty = -1;

        vis_certainty = (vis_certainty + 1) / (visualisation_boundary + 1);

        // A rough heuristic for box around the face width
        int thickness = (int)std::ceil(2.0* ((double)captured_image.cols) / 640.0);

        Vec6d pose_estimate_to_draw = CLMTracker::GetCorrectedPoseWorld(clm_model, fx, fy, cx, cy);

        // Draw it in reddish if uncertain, blueish if certain
        CLMTracker::DrawBox(captured_image, pose_estimate_to_draw, Scalar((1 - vis_certainty)*255.0, 0, vis_certainty * 255), thickness, fx, fy, cx, cy);

    }
}

void runCLM(Mat &captured_image,Mat &output_image){


    // If optical centers are not defined just use center of image

    cx = captured_image.cols / 2.0f;
    cy = captured_image.rows / 2.0f;
    // Use a rough guess-timate of focal length

    fx = 500 * (captured_image.cols / 640.0);
    fy = 500 * (captured_image.rows / 480.0);

    fx = (fx + fy) / 2.0;
    fy = fx;
    // Grab the timestamp first
    if (fps_vid_in == -1)
    {
        int64 curr_time = cv::getTickCount();
        time_stamp = (double(curr_time - t_initial) / cv::getTickFrequency());
    }
    else
    {
        time_stamp = (double)frame_count * (1.0 / fps_vid_in);
    }

    // Reading the images
    Mat_<float> depth_image;
    Mat_<uchar> grayscale_image;
    if (captured_image.channels() == 3)
    {
        cvtColor(captured_image, grayscale_image, CV_BGR2GRAY);
    }
    else
    {
        grayscale_image = captured_image.clone();
    }
    // The actual facial landmark detection / tracking
    long st=getCurrentTime();
    LOGD("Starting tracking");
    bool detection_success = CLMTracker::DetectLandmarksInVideo(grayscale_image, depth_image, *clm_model, *clm_parameters);
    // Work out the pose of the head from the tracked model
    visualise_tracking(output_image, depth_image, *clm_model, *clm_parameters, frame_count, fx, fy, cx, cy);
    LOGD("End tracking,state:%d %.3lf,time :%ld",(int)detection_success,clm_model->detection_certainty,getCurrentTime()-st);

    // Work out the framerate
    if (frame_count % 10 == 0)
    {
        double t1 = cv::getTickCount();
        fps_tracker = 10.0 / (double(t1 - t0) / cv::getTickFrequency());
        t0 = t1;
    }

    // Write out the framerate on the image before displaying it
    char fpsC[255];
    std::sprintf(fpsC, "%d", (int)fps_tracker);
    string fpsSt("FPS:");
    fpsSt += fpsC;
    cv::putText(output_image, fpsSt, cv::Point(10, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));

    // Update the frame count
    frame_count++;

}

void Java_com_martin_ads_clm4android_MainActivity_nativeProcessFrame(JNIEnv*, jobject, jlong addrGray, jlong addrRgba) {
    Mat &mGr = *(Mat *) addrGray;
    Mat &mRgba = *(Mat *) addrRgba;

    runCLM(mGr,mRgba);

/*    LOGD("%d image size %d",mGr.rows,mGr.cols);
    resize(mGr,mGr,Size(mGr.cols*480/mGr.rows,480));
    LOGD("%d image new-size %d",mGr.rows,mGr.cols);*/

}

JNIEXPORT jboolean JNICALL
Java_com_martin_ads_clm4android_MainActivity_initCLM(JNIEnv *env, jobject instance, jstring path_) {
    const char *path = env->GetStringUTFChars(path_, 0);

    setModelPath(path);

    clm_parameters=new CLMTracker::CLMParameters;
    LOGD("%s",clm_parameters->model_location.c_str());

    clm_model=new CLMTracker::CLM(clm_parameters->model_location);


    fx = 0, fy = 0, cx = 0, cy = 0;

    fps_vid_in = -1.0;

    frame_count = 0;

    // Use for timestamping if using a webcam
    t_initial = cv::getTickCount();

    // Timestamp in seconds of current processing
    time_stamp = 0;

    LOGD("clm init finish");
    env->ReleaseStringUTFChars(path_, path);
    return true;
}


}


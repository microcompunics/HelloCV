
//  Header for all external CLMModel/CLNF/CLMModel-Z methods of interest to the user
//
//
//  Tadas Baltrusaitis
//  01/05/2012
#ifndef __CLM_TRACKER_h_
#define __CLM_TRACKER_h_

#include "common.h"
#include "CLMParameters.h"
#include "CLM_utils.h"
#include "CLM.h"

using namespace std;
using namespace cv;

namespace CLMTracker
{

	//================================================================================================================
	// Landmark detection in videos, need to provide an image and model parameters (default values work well)
	// Optionally can provide a bounding box from which to start tracking
	//================================================================================================================
	bool DetectLandmarksInVideo(const Mat_<uchar> &grayscale_image, CLM& clm_model, CLMParameters& params);
	bool DetectLandmarksInVideo(const Mat_<uchar> &grayscale_image, const Mat_<float> &depth_image, CLM& clm_model, CLMParameters& params);

	bool DetectLandmarksInVideo(const Mat_<uchar> &grayscale_image, const Rect_<double> bounding_box, CLM& clm_model, CLMParameters& params);
	bool DetectLandmarksInVideo(const Mat_<uchar> &grayscale_image, const Mat_<float> &depth_image, const Rect_<double> bounding_box, CLM& clm_model, CLMParameters& params);

	//================================================================================================================
	// Landmark detection in image, need to provide an image and optionally CLMModel model together with parameters (default values work well)
	// Optionally can provide a bounding box in which detection is performed (this is useful if multiple faces are to be detected in images)
	//================================================================================================================
	bool DetectLandmarksInImage(const Mat_<uchar> &grayscale_image, CLM& clm_model, CLMParameters& params);
	// Providing a bounding box
	bool DetectLandmarksInImage(const Mat_<uchar> &grayscale_image, const Rect_<double> bounding_box, CLM& clm_model, CLMParameters& params);

	//================================================
	// CLMModel-Z versions
	bool DetectLandmarksInImage(const Mat_<uchar> &grayscale_image, const Mat_<float> depth_image, CLM& clm_model, CLMParameters& params);
	bool DetectLandmarksInImage(const Mat_<uchar> &grayscale_image, const Mat_<float> depth_image, const Rect_<double> bounding_box, CLM& clm_model, CLMParameters& params);

	//================================================================
	// Helper function for getting head pose from CLMModel parameters

	// Return the current estimate of the head pose, this can be either in camera or world coordinate space
	// The format returned is [Tx, Ty, Tz, Eul_x, Eul_y, Eul_z]
	Vec6d GetPoseCamera(const CLM& clm_model, double fx, double fy, double cx, double cy);
	Vec6d GetPoseWorld(const CLM& clm_model, double fx, double fy, double cx, double cy);
	
	// Getting a head pose estimate from the currently detected landmarks, with appropriate correction for perspective
	// This is because rotation estimate under orthographic assumption is only correct close to the centre of the image
	// These methods attempt to correct for that
	// The pose returned can be either in camera or world coordinates
	// The format returned is [Tx, Ty, Tz, Eul_x, Eul_y, Eul_z]
	Vec6d GetCorrectedPoseCamera(const CLM& clm_model, double fx, double fy, double cx, double cy);
	Vec6d GetCorrectedPoseWorld(const CLM& clm_model, double fx, double fy, double cx, double cy);

	//===========================================================================

}
#endif


//  Header for all external CLMModel methods of interest to the user
//
//
//  Tadas Baltrusaitis
//  28/03/2014
#ifndef __CLM_UTILS_h_
#define __CLM_UTILS_h_

#include "common.h"
#include "CLM.h"

using namespace std;
using namespace cv;

namespace CLMTracker
{
	//===========================================================================	
	// Defining a set of useful utility functions to be used within CLMModel


	//=============================================================================================
	// Helper functions for parsing the inputs
	//=============================================================================================

	void get_camera_params(int &device, float &fx, float &fy, float &cx, float &cy, vector<string> &arguments);

	//===========================================================================
	// Fast patch expert response computation (linear model across a ROI) using normalised cross-correlation
	//===========================================================================
	// This is a modified version of openCV code that allows for precomputed dfts of templates and for precomputed dfts of an image
	// _img is the input img, _img_dft it's dft (optional), _integral_img the images integral image (optional), squared integral image (optional), 
	// templ is the template we are convolving with, templ_dfts it's dfts at varying windows sizes (optional),  _result - the output, method the type of convolution
	void matchTemplate_m( const Mat_<float>& input_img, Mat_<double>& img_dft, cv::Mat& _integral_img, cv::Mat& _integral_img_sq, const Mat_<float>&  templ, map<int, Mat_<double> >& templ_dfts, Mat_<float>& result, int method );

	//===========================================================================
	// Point set and landmark manipulation functions
	//===========================================================================
	// Using Kabsch's algorithm for aligning shapes
	//This assumes that align_from and align_to are already mean normalised
	Matx22d AlignShapesKabsch2D(const Mat_<double>& align_from, const Mat_<double>& align_to );

	//=============================================================================
	// Basically Kabsch's algorithm but also allows the collection of points to be different in scale from each other
	Matx22d AlignShapesWithScale(cv::Mat_<double>& src, cv::Mat_<double> dst);

	//===========================================================================
	// Visualisation functions
	//===========================================================================
	void Project(Mat_<double>& dest, const Mat_<double>& mesh, double fx, double fy, double cx, double cy);
	void DrawBox(Mat image, Vec6d pose, Scalar color, int thickness, float fx, float fy, float cx, float cy);

	// Drawing face bounding box
	vector<pair<Point, Point>> CalculateBox(Vec6d pose, float fx, float fy, float cx, float cy);
	void DrawBox(vector<pair<Point, Point>> lines, Mat image, Scalar color, int thickness);

	vector<Point2d> CalculateLandmarks(const Mat_<double>& shape2D, Mat_<int>& visibilities);
	vector<Point2d> CalculateLandmarks(const Mat_<double>& shape2D);
	vector<Point2d> CalculateLandmarks(CLM& clm_model);
	void DrawLandmarks(cv::Mat img, vector<Point> landmarks);

	void Draw(cv::Mat img, const Mat_<double>& shape2D, const Mat_<int>& visibilities);
	void Draw(cv::Mat img, const Mat_<double>& shape2D);
	void Draw(cv::Mat img, const CLM& clm_model);


	//===========================================================================
	// Angle representation conversion helpers
	//===========================================================================
	Matx33d Euler2RotationMatrix(const Vec3d& eulerAngles);

	// Using the XYZ convention R = Rx * Ry * Rz, left-handed positive sign
	Vec3d RotationMatrix2Euler(const Matx33d& rotation_matrix);

	Vec3d Euler2AxisAngle(const Vec3d& euler);

	Vec3d AxisAngle2Euler(const Vec3d& axis_angle);

	Matx33d AxisAngle2RotationMatrix(const Vec3d& axis_angle);

	Vec3d RotationMatrix2AxisAngle(const Matx33d& rotation_matrix);

	//============================================================================
	// Face detection helpers
	//============================================================================

	// Face detection using Haar cascade classifier
	bool DetectFaces(vector<Rect_<double> >& o_regions, const Mat_<uchar>& intensity);
	bool DetectFaces(vector<Rect_<double> >& o_regions, const Mat_<uchar>& intensity, CascadeClassifier& classifier);
	// The preference point allows for disambiguation if multiple faces are present (pick the closest one), if it is not set the biggest face is chosen
	bool DetectSingleFace(Rect_<double>& o_region, const Mat_<uchar>& intensity, CascadeClassifier& classifier, const cv::Point preference = Point(-1,-1));

//	// Face detection using HOG-SVM classifier
//	bool DetectFacesHOG(vector<Rect_<double> >& o_regions, const Mat_<uchar>& intensity, std::vector<double>& confidences);
//	bool DetectFacesHOG(vector<Rect_<double> >& o_regions, const Mat_<uchar>& intensity, dlib::frontal_face_detector& classifier, std::vector<double>& confidences);
//	// The preference point allows for disambiguation if multiple faces are present (pick the closest one), if it is not set the biggest face is chosen
//	bool DetectSingleFaceHOG(Rect_<double>& o_region, const Mat_<uchar>& intensity, dlib::frontal_face_detector& classifier, double& confidence, const cv::Point preference = Point(-1,-1));

	//============================================================================
	// Matrix reading functionality
	//============================================================================

	// Reading a matrix written in a binary format
	void ReadMatBin(std::ifstream& stream, Mat &output_mat);

	// Reading in a matrix from a stream
	void ReadMat(std::ifstream& stream, Mat& output_matrix);

	// Skipping comments (lines starting with # symbol)
	void SkipComments(std::ifstream& stream);

}
#endif

#ifndef __PDM_h_
#define __PDM_h_

#include "CLMParameters.h"

using namespace cv;

namespace CLMTracker
{
//===========================================================================
// A linear 3D Point Distribution Model (constructed using Non-Rigid structure from motion or PCA)
// Only describes the model but does not contain an instance of it (no local or global parameters are stored here)
// Contains the utility functions to help manipulate the model

class PDM{
	public:    
    
		// The 3D mean shape vector of the PDM [x1,..,xn,y1,...yn,z1,...,zn]
		cv::Mat_<double> mean_shape;	
  
		// Principal components or variation bases of the model, 
		cv::Mat_<double> princ_comp;	

		// Eigenvalues (variances) corresponding to the bases
		cv::Mat_<double> eigen_values;	

		PDM(){;}
		
		// A copy constructor
		PDM(const PDM& other){
			
			// Make sure the matrices are allocated properly
			this->mean_shape = other.mean_shape.clone();
			this->princ_comp = other.princ_comp.clone();
			this->eigen_values = other.eigen_values.clone();
		}
			
		void Read(string location);

		// Number of vertices
		inline int NumberOfPoints() const {return mean_shape.rows/3;}
		
		// Listing the number of modes of variation
		inline int NumberOfModes() const {return princ_comp.cols;}

		void Clamp(Mat_<float>& params_local, Vec6d& params_global, const CLMParameters& params);

		// Compute shape in object space (3D)
		void CalcShape3D(Mat_<double>& out_shape, const Mat_<double>& params_local) const;

		// Compute shape in image space (2D)
		void CalcShape2D(Mat_<double>& out_shape, const Mat_<double>& params_local, const Vec6d& params_global) const;
    
		// provided the bounding box of a face and the local parameters (with optional rotation), generates the global parameters that can generate the face with the provided bounding box
		void CalcParams(Vec6d& out_params_global, const Rect_<double>& bounding_box, const Mat_<double>& params_local, const Vec3d rotation = Vec3d(0.0));

		// Provided the landmark location compute global and local parameters best fitting it (can provide optional rotation for potentially better results)
		void CalcParams(Vec6d& out_params_global, const Mat_<double>& out_params_local, const Mat_<double>& landmark_locations, const Vec3d rotation = Vec3d(0.0));

		// provided the model parameters, compute the bounding box of a face
		void CalcBoundingBox(Rect& out_bounding_box, const Vec6d& params_global, const Mat_<double>& params_local);

		// Helpers for computing Jacobians, and Jacobians with the weight matrix
		void ComputeRigidJacobian(const Mat_<float>& params_local, const Vec6d& params_global, Mat_<float> &Jacob, const Mat_<float> W, cv::Mat_<float> &Jacob_t_w);
		void ComputeJacobian(const Mat_<float>& params_local, const Vec6d& params_global, Mat_<float> &Jacobian, const Mat_<float> W, cv::Mat_<float> &Jacob_t_w);

		// Given the current parameters, and the computed delta_p compute the updated parameters
		void UpdateModelParameters(const Mat_<float>& delta_p, Mat_<float>& params_local, Vec6d& params_global);

  };
  //===========================================================================
}
#endif

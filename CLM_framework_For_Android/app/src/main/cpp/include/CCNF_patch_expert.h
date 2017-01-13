
#ifndef __CCNF_PATCH_EXPERT_h_
#define __CCNF_PATCH_EXPERT_h_

#include "common.h"

using namespace cv;

namespace CLMTracker
{

//===========================================================================
/** 
	A single Neuron response
*/
class CCNF_neuron{

public:

	// Type of patch (0=raw,1=grad,3=depth, other types besides raw are not actually used now)
	int     neuron_type; 

	// scaling of weights (needed as the energy of neuron might not be 1) 
	double  norm_weights; 

	// Weight bias
	double  bias;

	// Neural weights
	cv::Mat_<float> weights; 

	// can have neural weight dfts that are calculated on the go as needed, this allows us not to recompute
	// the dft of the template each time, improving the speed of tracking
	std::map<int, cv::Mat_<double> > weights_dfts;

	// the alpha associated with the neuron
	double alpha; 

	// Default constructor
	CCNF_neuron(){;}

	// Copy constructor
	CCNF_neuron(const CCNF_neuron& other): weights(other.weights.clone())
	{
		this->neuron_type = other.neuron_type;
		this->norm_weights = other.norm_weights;
		this->bias = other.bias;
		this->alpha = other.alpha;

		for(std::map<int, Mat_<double> >::const_iterator it = other.weights_dfts.begin(); it!= other.weights_dfts.end(); it++)
		{
			// Make sure the matrix is copied.
			this->weights_dfts.insert(std::pair<int, Mat>(it->first, it->second.clone()));
		}
	}

	void Read(std::ifstream &stream);
	// The im_dft, integral_img, and integral_img_sq are precomputed images for convolution speedups (they get set if passed in empty values)
	void Response(Mat_<float> &im, Mat_<double> &im_dft, Mat &integral_img, Mat &integral_img_sq, Mat_<float> &resp);

};

//===========================================================================
/**
A CCNF patch expert
*/
class CCNF_patch_expert{
public:
    
	// Width and height of the patch expert support region
	int width;
	int height;             
    
	// Collection of neurons for this patch expert
	std::vector<CCNF_neuron> neurons;

	// Information about the vertex features (association potentials)
	std::vector<int>				window_sizes;
	std::vector<cv::Mat_<float> >	Sigmas;
	std::vector<double>				betas;

	// How confident we are in the patch
	double   patch_confidence;

	// Default constructor
	CCNF_patch_expert(){;}

	// Copy constructor		
	CCNF_patch_expert(const CCNF_patch_expert& other): neurons(other.neurons), window_sizes(other.window_sizes), betas(other.betas)
	{
		this->width = other.width;
		this->height = other.height;
		this->patch_confidence = other.patch_confidence;

		// Copy the Sigmas in a deep way
		for(std::vector<Mat_<float> >::const_iterator it = other.Sigmas.begin(); it!= other.Sigmas.end(); it++)
		{
			// Make sure the matrix is copied.
			this->Sigmas.push_back(it->clone());
		}

	}


	void Read(std::ifstream &stream, std::vector<int> window_sizes, std::vector<std::vector<Mat_<float> > > sigma_components);

	// actual work (can pass in an image and a potential depth image, if the CCNF is trained with depth)
	void Response(Mat_<float> &area_of_interest, Mat_<float> &response);    

	// Helper function to compute relevant sigmas
	void ComputeSigmas(std::vector<Mat_<float> > sigma_components, int window_size);
	
};
  //===========================================================================
}
#endif

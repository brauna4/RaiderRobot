/*
 * humandetect.h
 *
 *  Created on: Aug 7, 2012
 *      Author: udvisionlab
 */

#ifndef BODY_DETECTOR_H
#define BODY_DETECTOR_H

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "descriptor.h"
#include <string>
#include <vector>
#include <cstdio>

using namespace cv;

#define F_THRESHOLD 0.1
#define C_THRESHOLD 0.75
#define SCALE_HEIGHT 430
#define SCALE_WIDTH 150
#define ALPHA 0.5

struct PersonProfile
{
	Rect location;
	vector<float> feature_vector; //this is used for identifying the detected individual
	vector<float> color_hist;
	bool found; 
	union
	{
		int id;
		//string name;
		int key;
	};
};

class BodyDetector
{
public:
	//parameters for the HOG descriptor
	Mat Image;
	Mat Image_ROI;
	Mat bodyMask; // fixed mask to remove considerable background in detected region
	HOGDescriptor* hog;
	gpu::HOGDescriptor* hog_gpu;
	
	//Initialize LBP Descriptor
	LBPDescriptor* lbp_descriptor;

	Size win_size;
	Size block_size;
	Size block_stride;
	Size cell_size;
	int nbins;
	int deriv_aperture;
	int histogram_NormType;
	double win_sigma;
	double threshold_L2hys;
	bool gamma_correction;
	int nlevels;

	//parameters for detection
	double hit_threshold; //hit threshold for detection
	Size win_stride;
	Size padding;
	double scale0;


	vector<float> svm_coeff; //svm coefficients
	vector<Rect> locations;
	bool useGPU;

	int num_detections;
	bool detected;
	
	int num_unique_detections;
	vector<PersonProfile> unique_detections;

public:
	BodyDetector(bool flag); //constructor
	~BodyDetector(); //desctructor
	vector<PersonProfile> DetectAndDrawHumanBody(Mat& frame);
	int CreateUniqueProfile(Mat img); //for each detection, a descriptor is computed and then correspondingly matched to every .. the feature matching is implemented in this function
	void DisplayID(Mat& img,PersonProfile pf); // for displaying the unique id or name
	float FeatureMatch(vector<float> f1,vector<float> f2);
	void UpdateFeature(vector<float>& model_fl,vector<float> query_fl);
	vector<float> ComputeColorHistogram(Mat img,Mat mask = Mat());
	void ClearUniqueDetections();
};


#endif

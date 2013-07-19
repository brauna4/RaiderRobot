/*
 * face_detector.h
 *
 *  Created on: Jul 27, 2012
 *      Author: bnair002
 */

#ifndef _FACE_DETECTOR_H_
#define _FACE_DETECTOR_H_

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <string>

using namespace cv;

#define FRONTALFACECASCADE_PATH "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt2.xml"
//#define FRONTALFACECASCADE_PATH "/usr/local/share/OpenCV/lbpcascades/lbpcascade_frontalface.xml"
#define PROFILEFACECASCADE_PATH "/usr/local/share/OpenCV/haarcascades/haarcascade_profileface.xml"

//structure containing the details of the face detected, such as his/her name, other facial features for tracking
struct FaceProfile
{
	Rect face_image;
	string id; //individuals name
};

//declaring the class for face detector
class FaceDetector
{
public:
	gpu::CascadeClassifier_GPU profileface_cascade;
	gpu::CascadeClassifier_GPU frontalface_cascade;

	vector<FaceProfile> face_profiles;

	//constructor/destructor
	FaceDetector();
	~FaceDetector();

	//methods
	vector<FaceProfile> detectFaces(Mat frame);
};

#endif /* FACE_DETECTOR_H_ */

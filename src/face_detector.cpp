/*
 * face_detector.cpp
 *
 *  Created on: Jul 27, 2012
 *      Author: bnair002
 */

#include "face_detector.h"

//define the class

//constructor/destructor
FaceDetector::FaceDetector()
{
	frontalface_cascade.load(FRONTALFACECASCADE_PATH);
	//profileface_cascade.load(PROFILEFACECASCADE_PATH);
}

FaceDetector::~FaceDetector()
{
	frontalface_cascade.release();
}

vector<FaceProfile> FaceDetector::detectFaces(Mat frame)
{
	Mat frame_small,frame_gray;
	pyrDown(frame,frame_small);

	face_profiles.clear();

	cvtColor(frame_small,frame_gray,CV_BGR2GRAY);
	equalizeHist(frame_gray,frame_gray);

	//loading the Mat onto the GPU
	gpu::GpuMat g_frame(frame_gray);
	gpu::GpuMat objBuf;

	//detecting faces
	int detection_number = frontalface_cascade.detectMultiScale(g_frame,objBuf,1.1,3);
	Mat obj_host;

	//download only detected number of rectangles
	objBuf.colRange(0,detection_number).download(obj_host);
	Rect* temp_faces = obj_host.ptr<Rect>();
	g_frame.download(frame);

	vector<Rect> faces(temp_faces,temp_faces + detection_number);

	vector<Rect>::iterator it;

	it = faces.begin();
	for(it = faces.begin(); it!= faces.end() ; it++)
	{
		FaceProfile face_p;
		Rect face = (*it);

		//increase the size of the rectangle by 2
		face = face + Size(face.width,face.height);
		face = face + Point(face.x,face.y);

		// get the interest region and perform recognition and get string result

		//store it in FaceProfile
		face_p.face_image = face;
		face_p.id = "UnKnown"; //recognition result

		face_profiles.push_back(face_p);
	}

	return face_profiles;
}



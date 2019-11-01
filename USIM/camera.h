#pragma once
#ifndef __CAMERA__
#define __CAMERA__

#include "common.h"

class Camera {
public:
	Camera();

	Camera(string camera_name);

	void loatMat();

	void getCameraSetting(string camera_name);

	void getMapMatrix();

	void printInformation();

	void getDisparityMap(string input_image_dir);

private:
	string name;
	cv::Mat CML, CMR, DML, DMR;
	cv::Mat R, T, E, F, R1, R2, T1, T2, Q;
	cv::Size image_size;
	cv::Mat map_x_L, map_y_L, map_x_R, map_y_R;
	cv::Ptr<cv::StereoSGBM> sgbm;
};




#endif
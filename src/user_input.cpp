/*
 * user_input.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#include "user_input.h"


void projectCloudIntoProjector(const Cloud& cloud, const cv::Mat& P, cv::Mat& img){

	cv::Mat p(4,1,CV_32FC1);
	cv::Mat px(3,1,CV_32FC1);

	int w = img.cols;
	int h = img.rows;

	img.setTo(0);

	float z;

	for (uint i=0; i<cloud.points.size(); ++i){
		p.at<float>(0) = cloud.points[i].x;
		p.at<float>(1) = cloud.points[i].y;
		p.at<float>(2) = cloud.points[i].z;
		p.at<float>(3) = 1;

		z = cloud.points[i].z;

		if (! z == z) continue;

		px = P*p;
		px /= px.at<float>(2);
		int x = px.at<float>(0);	int y = px.at<float>(1);
		if (x<0 || x >= w || y < 0 || y >= h)
			continue;

		//HACK: rather change whole system
		z = -z;

		if (z<0.03) continue;


		float z_max = 0.5;

		cv::Scalar col(z/z_max*180,255,255);

		cv::circle(img, cv::Point(x,y), 2, col,-1);

	}

	cv::cvtColor(img,img,CV_HSV2BGR);


}


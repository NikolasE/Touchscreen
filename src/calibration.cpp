/*
 * calibration.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#include "calibration.h"

void computeHomography(const vector<CvPoint2D32f>& corners_2d, const Cloud& corners_3d, CvMat* H){


	uint N = corners_3d.points.size();
	assert(N == corners_2d.size());

	// count number of 3d points which are more than 2cm away from the z=0-plane
	float z_max = 0.02; int cnt = 0;
	for (uint i=0; i<N; ++i) { if (abs(corners_3d.at(i).z) > z_max) cnt++; }
	if (cnt>N*0.01) {	ROS_WARN("found %f %% points with dist > 2cm", cnt*100.0/N); }



	// create Matrices
	CvMat* src = cvCreateMat(2,N,CV_32FC1);
	CvMat* dst = cvCreateMat(2,N,CV_32FC1);

	for (uint i=0; i<N; ++i){
		cvSet2D(src,0,i,cvScalarAll(corners_2d.at(i).x));
		cvSet2D(src,1,i,cvScalarAll(corners_2d.at(i).y));

		cvSet2D(dst,0,i,cvScalarAll(corners_3d.at(i).x));
		cvSet2D(dst,1,i,cvScalarAll(corners_3d.at(i).y));
	}


	cvFindHomography(src, dst, H, 0); // use default mode with no outlier handling

}



void drawCheckerboard(IplImage* img, const IplImage* mask, int w, int h, vector<CvPoint2D32f>& corners_2d){

	// get region of checkerboard
	float minx,maxx, miny, maxy;
	minx = miny = 1e5; maxx = maxy = -1e5;
	for (int i=0; i<mask->width; ++i)
		for (int j=0; j<mask->height; ++j){
			if (cvGet2D(mask,j,i).val[0] == 0) continue;
			minx = min(minx,i*1.f); miny = min(miny,j*1.f);
			maxx = max(maxx,i*1.f); maxy = max(maxy,j*1.f);
		}

//	ROS_INFO("x: %f %f, y: %f %f", minx,maxx, miny, maxy);

	float width = (maxx-minx)/(w+1);
	float height = (maxy-miny)/(h+1);

//	cvSet(img, cvScalarAll(255)); // all white

	// start with black square
	for (int j = 0; j<=h; j++)
		for (int i = (j%2); i<w+1; i+=2){
			CvPoint lu = cvPoint(minx+i*width,miny+j*height);
			CvPoint rl = cvPoint(minx+(i+1)*width,miny+(j+1)*height);
			cvRectangle(img, lu, rl ,cvScalarAll(0), -1);
			corners_2d.push_back(cvPoint2D32f(rl.x, rl.y));
		}

}

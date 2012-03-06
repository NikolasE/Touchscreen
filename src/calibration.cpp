/*
 * calibration.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#include "calibration.h"


void computeProjectionMatrix(const Cloud& corners, const vector<CvPoint2D32f>& projector_corners){

	uint c_cnt = corners.points.size();
	uint proj_cnt = projector_corners.size();

	assert(c_cnt % proj_cnt == 0);

	int img_cnt = c_cnt/proj_cnt;

	ROS_INFO("Computing Projection matrix from %i images", img_cnt);

	cv::Mat A = cv::Mat(2*c_cnt,9,CV_32FC1);


	for (uint i=0; i<c_cnt; ++i){



	}




	// TODO: normalize



}


void applyHomography(const CvPoint2D32f& p,const CvMat* H, CvPoint& p_){

	CvMat* pc = cvCreateMat(3,1,CV_32FC1);
	cvSet1D(pc,0,cvScalarAll(p.x));
	cvSet1D(pc,1,cvScalarAll(p.y));
	cvSet1D(pc,2,cvScalarAll(1));

	CvMat* p_proj = cvCreateMat(3,1,CV_32FC1);

	cvMatMul(H, pc,p_proj);

	p_.x = cvGet1D(p_proj,0).val[0];
	p_.y = cvGet1D(p_proj,1).val[0];
}


void computeHomography(const vector<CvPoint2D32f>& corners_2d, const Cloud& corners_3d, CvMat* H){


	assert(H &&  H->cols == 3);

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
		cvSet2D(src,0,i,cvScalarAll(corners_3d.at(i).x));
		cvSet2D(src,1,i,cvScalarAll(corners_3d.at(i).y));

		cvSet2D(dst,0,i,cvScalarAll(corners_2d.at(i).x));
		cvSet2D(dst,1,i,cvScalarAll(corners_2d.at(i).y));

		//		printf("from %f %f to %f %f \n", corners_3d.at(i).x,corners_3d.at(i).y,corners_2d.at(i).x,corners_2d.at(i).y);

	}

	// 2d = H*3d H*(x,y,1)

	cvFindHomography(src, dst, H, 0); // use default mode with no outlier handling


}



void drawCheckerboard(IplImage* img, const IplImage* mask, CvSize size, vector<CvPoint2D32f>& corners_2d){

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


	// draw white border with this size
	// "Note: the function requires some white space (like a square-thick border,
	//	the wider the better) around the board to make the detection more robust in various environment"
	float border = 40;

	float width = (maxx-minx-2*border)/(size.width+1);
	float height = (maxy-miny-2*border)/(size.height+1);

	//	ROS_INFO("w,h: %f %f", width, height);




	cvSet(img, cvScalarAll(255)); // all white

	minx += border;
	miny += border;

	// start with black square
	for (int j = 0; j<=size.height; j++)
		for (int i = (j%2); i<size.width+1; i+=2){

			CvPoint lu = cvPoint(minx+i*width,miny+j*height);
			CvPoint rl = cvPoint(minx+(i+1)*width,miny+(j+1)*height);
			cvRectangle(img, lu, rl ,cvScalarAll(0), -1);


			CvPoint ru = cvPoint(rl.x,lu.y);

			if (j==0) continue;

			if (i>0){
				corners_2d.push_back(cvPoint2D32f(lu.x, lu.y));
				//				cvCircle(img, cvPoint(lu.x, lu.y),20, CV_RGB(255,0,0),3);
			}

			if (i<size.width){
				corners_2d.push_back(cvPoint2D32f(ru.x, ru.y));
				//				cvCircle(img, cvPoint(ru.x, ru.y),20, CV_RGB(255,0,0),3);
			}

		}



	assert(int(corners_2d.size()) == size.width*size.height);

}

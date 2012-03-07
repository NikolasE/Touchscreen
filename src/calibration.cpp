/*
 * calibration.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#include "calibration.h"

void scalePixels(const vector<CvPoint2D32f>& pxs, cv::Mat& T, vector<CvPoint2D32f>& transformed){

	uint c_cnt = pxs.size();

	Eigen::Vector2f mu(0,0);
	for (uint i=0; i<c_cnt; ++i){
		cv::Point2f px = pxs.at(i);
		mu[0] += px.x; mu[1] += px.y;
	}
	mu /= c_cnt;
	// get mean distance to center:
	double d = 0;
	for (uint i=0; i<c_cnt; ++i){
		cv::Point2f px = pxs.at(i);
		d += sqrt(pow(px.x-mu[0],2)+pow(px.y-mu[1],2));
	}
	d /= c_cnt;
//	ROS_INFO("2d: mean: %f %f, dist: %f", mu[0], mu[1], d);

	// mean distance should be sqrt(2)
	double s = sqrt(2)/d;

	T = cv::Mat::eye(3,3,CV_32FC1); T*=s;  T.at<float>(2,2) = 1;
	for (int i=0; i<2; ++i)
		T.at<float>(i,2) = -mu[i]*s;

	//	cout << "T" << endl << T << endl;

	// apply matrix on all points
	Cloud d3_scaled;

	cv::Mat P = cv::Mat(3,1,CV_32FC1);

	transformed.reserve(c_cnt);
	for (uint i=0; i<c_cnt; ++i){
		cv::Point2f p = pxs.at(i);

		P.at<float>(0,0) = p.x;
		P.at<float>(1,0) = p.y;
		P.at<float>(2,0) = 1;

		P = T*P;

		p.x = P.at<float>(0,0);
		p.y = P.at<float>(1,0);

		transformed.push_back(p);
	}


}



void scaleCloud(const Cloud& pts, cv::Mat& U, Cloud& transformed){

	uint c_cnt = pts.points.size();

	// normalize since metric coordinates are in [0,1]^3
	// and pixels in [0,640]x[0,480]
	Eigen::Vector3f mu(0,0,0);
	for (uint i=0; i<c_cnt; ++i){
		pcl_Point	p = pts.points.at(i);
		mu[0] += p.x; mu[1] += p.y; mu[2] += p.z;
	}
	mu /= c_cnt;
	// get mean distance to center:
	double d = 0;
	for (uint i=0; i<c_cnt; ++i){
		pcl_Point	p = pts.points.at(i);
		d += sqrt(pow(p.x-mu[0],2)+pow(p.y-mu[1],2)+pow(p.z-mu[2],2));
	}
	d /= c_cnt;
//	ROS_INFO("3d: mean: %f %f %f, dist: %f", mu[0], mu[1], mu[2], d);

	// mean distance should be sqrt(3)
	double s = sqrt(3)/d;



	U = cv::Mat::eye(4,4,CV_32FC1); U*=s;  U.at<float>(3,3) = 1;
	for (int i=0; i<3; ++i)
		U.at<float>(i,3) = -mu[i]*s;

	//	 cout << "U" << endl << U << endl;

	// apply matrix on all points
	Cloud d3_scaled;

	cv::Mat P = cv::Mat(4,1,CV_32FC1);

	transformed.reserve(c_cnt);
	for (uint i=0; i<c_cnt; ++i){
		pcl_Point		p = pts.points.at(i);

		P.at<float>(0,0) = p.x;
		P.at<float>(1,0) = p.y;
		P.at<float>(2,0) = p.z;
		P.at<float>(3,0) = 1;

		P = U*P;

		p.x = P.at<float>(0,0);
		p.y = P.at<float>(1,0);
		p.z = P.at<float>(2,0);

		transformed.push_back(p);
	}


}


void computeProjectionMatrix(cv::Mat& P ,const Cloud& corners, const vector<CvPoint2D32f>& projector_corners){

	uint c_cnt = corners.points.size();
	uint proj_cnt = projector_corners.size();


	assert(c_cnt >0);
	assert(c_cnt % proj_cnt == 0);

	int img_cnt = c_cnt/proj_cnt;

	ROS_INFO("Computing Projection matrix from %i images", img_cnt);


	Cloud trafoed_corners;
	vector<CvPoint2D32f> trafoed_px;
	cv::Mat U,T;

	scaleCloud(corners, U, trafoed_corners);
	scalePixels(projector_corners, T, trafoed_px);

	cv::Mat A = cv::Mat(2*c_cnt,12,CV_32FC1);
	A.setTo(0);



	// p_ cross H*p = 0
	for (uint i=0; i<c_cnt; ++i){
		pcl_Point		P = trafoed_corners.points.at(i);
		cv::Point2f p = trafoed_px.at(i%proj_cnt);

		float f[12] = {0,0,0,0,-P.x,-P.y,-P.z,1,p.y*P.x,p.y*P.y,p.y*P.z,p.y};
		for (uint j=0; j<12; ++j)	A.at<float>(2*i,j) = f[j];

		float g[12] = {P.x,P.y,P.z,1,0,0,0,0,-p.x*P.x,-p.x*P.y,-p.x*P.z,-p.x};
		for (uint j=0; j<12; ++j)	A.at<float>(2*i+1,j) = g[j];
	}

	// now solve A*h == 0
	// Solution is the singular vector with smallest singular value

	cv::Mat h = cv::Mat(12,1,CV_32FC1);
	cv::SVD::solveZ(A,h);

	P = cv::Mat(3,4,CV_32FC1);

	for (uint i=0; i<3; ++i){
		for (uint j=0; j<4; ++j)
			P.at<float>(i,j) =  h.at<float>(4*i+j);
	}


//	cout << "Tinv " << T.inv() << endl;
//	cout << "U " << U << endl;


	// undo scaling
	P = T.inv()*P*U;


//	cout << "P " <<  P << endl;


	// compute reprojection error:
		double total = 0;

	cv::Mat P4 = cv::Mat(4,1,CV_32FC1);
	cv::Mat P3 = cv::Mat(3,1,CV_32FC1);


	for (uint i=0; i<c_cnt; ++i){
//		ROS_INFO("projection %i", i);

		pcl_Point		p = corners.points.at(i);
		cv::Point2f p_ = projector_corners.at(i%proj_cnt);

		P4.at<float>(0,0) = p.x;
		P4.at<float>(1,0) = p.y;
		P4.at<float>(2,0) = p.z;
		P4.at<float>(3,0) = 1;

//		cout << "P4 " <<  P4 << endl;
		P3 = P*P4;
		P3 /= P3.at<float>(2);
//		cout << "projected: " << P3 << endl;
//
//		cout << "pixel: " << p_.x << " " << p_.y << endl;
//		cout << endl;


		total += sqrt(pow(P3.at<float>(0)-p_.x,2)+pow(P3.at<float>(1)-p_.y,2));

	}

	total /= c_cnt;

	ROS_INFO("mean error: %f", total);

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

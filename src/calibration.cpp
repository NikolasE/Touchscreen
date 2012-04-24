/*
 * calibration.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#include "calibration.h"

void scalePixels(const vector<cv::Point2f>& pxs, cv::Mat& T, vector<cv::Point2f>& transformed){

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

 T = cv::Mat::eye(3,3,CV_64FC1); T*=s;  T.at<double>(2,2) = 1;
 for (int i=0; i<2; ++i)
  T.at<double>(i,2) = -mu[i]*s;

 //	cout << "T" << endl << T << endl;

 // apply matrix on all points
 Cloud d3_scaled;

 cv::Mat P = cv::Mat(3,1,CV_64FC1);

 transformed.reserve(c_cnt);
 for (uint i=0; i<c_cnt; ++i){
  cv::Point2f p = pxs.at(i);

  P.at<double>(0,0) = p.x;
  P.at<double>(1,0) = p.y;
  P.at<double>(2,0) = 1;

  P = T*P;

  p.x = P.at<double>(0,0);
  p.y = P.at<double>(1,0);

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



 U = cv::Mat::eye(4,4,CV_64FC1); U*=s;  U.at<double>(3,3) = 1;
 for (int i=0; i<3; ++i)
  U.at<double>(i,3) = -mu[i]*s;

 //	 cout << "U" << endl << U << endl;

 // apply matrix on all points
 Cloud d3_scaled;

 cv::Mat P = cv::Mat(4,1,CV_64FC1);

 transformed.reserve(c_cnt);
 for (uint i=0; i<c_cnt; ++i){
  pcl_Point		p = pts.points.at(i);

  P.at<double>(0,0) = p.x;
  P.at<double>(1,0) = p.y;
  P.at<double>(2,0) = p.z;
  P.at<double>(3,0) = 1;

  P = U*P;

  p.x = P.at<double>(0,0);
  p.y = P.at<double>(1,0);
  p.z = P.at<double>(2,0);

  transformed.push_back(p);
 }


}

void computeProjectionMatrix(cv::Mat& P ,const Cloud& corners, const vector<cv::Point2f>& projector_corners){

 uint c_cnt = corners.points.size();
 uint proj_cnt = projector_corners.size();


 assert(c_cnt >0);
 assert(c_cnt % proj_cnt == 0);

 int img_cnt = c_cnt/proj_cnt;

 ROS_INFO("Computing Projection matrix from %i images", img_cnt);


 Cloud trafoed_corners;
 vector<cv::Point2f> trafoed_px;
 cv::Mat U,T;

 scaleCloud(corners, U, trafoed_corners);
 scalePixels(projector_corners, T, trafoed_px);

 cv::Mat A = cv::Mat(2*c_cnt,12,CV_64FC1);
 A.setTo(0);

 // p_ cross H*p = 0
 for (uint i=0; i<c_cnt; ++i){
  pcl_Point		P = trafoed_corners.points.at(i);
  cv::Point2f p = trafoed_px.at(i%proj_cnt);

  float f[12] = {0,0,0,0,-P.x,-P.y,-P.z,1,p.y*P.x,p.y*P.y,p.y*P.z,p.y};
  for (uint j=0; j<12; ++j)	A.at<double>(2*i,j) = f[j];

  float g[12] = {P.x,P.y,P.z,1,0,0,0,0,-p.x*P.x,-p.x*P.y,-p.x*P.z,-p.x};
  for (uint j=0; j<12; ++j)	A.at<double>(2*i+1,j) = g[j];
 }

 // now solve A*h == 0
 // Solution is the singular vector with smallest singular value

 cv::Mat h = cv::Mat(12,1,CV_64FC1);
 cv::SVD::solveZ(A,h);

 P = cv::Mat(3,4,CV_64FC1);

 for (uint i=0; i<3; ++i){
  for (uint j=0; j<4; ++j)
   P.at<double>(i,j) =  h.at<double>(4*i+j);
 }

// cout << "Tinv " << T.inv() << endl;
// cout << "U " << U << endl;

 // undo scaling
 P = T.inv()*P*U;


 //	cout << "P " <<  P << endl;


 // compute reprojection error:
 double total = 0;
 cv::Point2f px;

 for (uint i=0; i<c_cnt; ++i){
  //		ROS_INFO("projection %i", i);

  pcl_Point		p = corners.points.at(i);
  cv::Point2f p_ = projector_corners.at(i%proj_cnt);

  applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),P,px);

  total += sqrt(pow(px.x-p_.x,2)+pow(px.y-p_.y,2));

 }

 total /= c_cnt;

 ROS_INFO("mean error: %f", total);

}


void applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P, cv::Point2f& p_){
 cv::Mat P4 = cv::Mat(4,1,CV_64FC1);
 cv::Mat P3 = cv::Mat(3,1,CV_64FC1);

 P4.at<double>(0,0) = p.x;
 P4.at<double>(1,0) = p.y;
 P4.at<double>(2,0) = p.z;
 P4.at<double>(3,0) = 1;

 P3 = P*P4;
 P3 /= P3.at<double>(2);

 p_.x = P3.at<double>(0);
 p_.y = P3.at<double>(1);

}



void applyHomography(const cv::Point2f& p,const cv::Mat& H, cv::Point2f& p_){

 cv::Mat pc (3,1,CV_64FC1);
 pc.at<double>(0) = p.x;
 pc.at<double>(1) = p.y;
 pc.at<double>(2) = 1;

 cv::Mat p_proj(3,1,CV_64FC1);

 p_proj = H*pc;

 p_.x = p_proj.at<double>(0);
 p_.y = p_proj.at<double>(1);
}


void computeHomography(const vector<cv::Point2f>& corners_2d, const Cloud& corners_3d, cv::Mat& H){


 uint N = corners_3d.points.size();
 assert(N == corners_2d.size());

 // count number of 3d points which are more than 2cm away from the z=0-plane
 float z_max = 0.02; int cnt = 0;
 for (uint i=0; i<N; ++i) { if (abs(corners_3d.at(i).z) > z_max) cnt++; }
 if (cnt>N*0.01) {	ROS_WARN("found %f %% points with dist > 2cm", cnt*100.0/N); }


 // create Matrices
 //	cv::Mat src(2,N,CV_32FC1);
 //	cv::Mat dst(2,N,CV_32FC1);

 vector<cv::Point2f> src; src.reserve(N);

 for (uint i=0; i<N; ++i){
  //		src.at<float>(0,i) = corners_3d.at(i).x;
  //		src.at<float>(1,i) = corners_3d.at(i).y;

  cv::Point2f p;
  p.x = corners_3d.at(i).x;
  p.y = corners_3d.at(i).y;

  src.push_back(p);

  //		dst.at<float>(0,i) = corners_2d.at(i).x;
  //		dst.at<float>(1,i) = corners_2d.at(i).y;

  //		cvSet2D(src,0,i,cvScalarAll(corners_3d.at(i).x));
  //		cvSet2D(src,1,i,cvScalarAll(corners_3d.at(i).y));
  //
  //		cvSet2D(dst,0,i,cvScalarAll(corners_2d.at(i).x));
  //		cvSet2D(dst,1,i,cvScalarAll(corners_2d.at(i).y));

  //		printf("from %f %f to %f %f \n", corners_3d.at(i).x,corners_3d.at(i).y,corners_2d.at(i).x,corners_2d.at(i).y);

 }

 // 2d = H*3d H*(x,y,1)

#define DO_SCALING

#ifdef DO_SCALING
 cv::Mat T,U;
 vector<cv::Point2f> src_trafoed, d2_trafoed;
 scalePixels(src,  T, src_trafoed);
 scalePixels(corners_2d,  U, d2_trafoed);
 H = cv::findHomography(src_trafoed,d2_trafoed);
 H = U.inv()*H*T;
#else
 H = cv::findHomography(src,corners_2d);
#endif

 // compute error:
 float error = 0;

 cv::Point2f px;
 for (uint i=0; i<N; ++i){
  applyHomography(cv::Point2f(corners_3d.at(i).x,corners_3d.at(i).y),H, px);

  float err = sqrt((px.x-corners_2d.at(i).x)*(px.x-corners_2d.at(i).x)+
    (px.y-corners_2d.at(i).y)*(px.y-corners_2d.at(i).y));

  error += err/N;

  ROS_INFO("Proj: %f %f, goal: %f %f (Error: %f)", px.x,px.y,corners_2d.at(i).x, corners_2d.at(i).y,err);
 }

 ROS_INFO("mean error: %f px", error);

 //	cvFindHomography(src, dst, H, 0); // use default mode with no outlier handling


}



void drawCheckerboard(cv::Mat* img, const cv::Mat* mask, cv::Size size, vector<cv::Point2f>& corners_2d){

 // get region of checkerboard
 float minx,maxx, miny, maxy;
 minx = miny = 1e5; maxx = maxy = -1e5;
 for (int i=0; i<mask->cols; ++i)
  for (int j=0; j<mask->rows; ++j){
   if (mask->at<uchar>(j,i) == 0) continue;
   minx = min(minx,i*1.f); miny = min(miny,j*1.f);
   maxx = max(maxx,i*1.f); maxy = max(maxy,j*1.f);
  }


 // draw white border with this size
 // "Note: the function requires some white space (like a square-thick border,
 //	the wider the better) around the board to make the detection more robust in various environment"
 float border = 80;

 float width = (maxx-minx-2*border)/(size.width+1);
 float height = (maxy-miny-2*border)/(size.height+1);

 img->setTo(255); // all white

 minx += border;
 miny += border;

 // start with black square
 for (int j = 0; j<=size.height; j++)
  for (int i = (j%2); i<size.width+1; i+=2){

   cv::Point2f lu = cv::Point2f(minx+i*width,miny+j*height);
   cv::Point2f rl = cv::Point2f(minx+(i+1)*width,miny+(j+1)*height);
   cv::rectangle(*img, lu, rl ,cv::Scalar::all(0), -1);

   cv::Point2f ru = cvPoint(rl.x,lu.y);

   if (j==0) continue;
   if (i>0){
    corners_2d.push_back(cv::Point2f(lu.x, lu.y));
    //				cvCircle(img, cvPoint(lu.x, lu.y),20, CV_RGB(255,0,0),3);
   }
   if (i<size.width){
    corners_2d.push_back(cv::Point2f(ru.x, ru.y));
    //				cvCircle(img, cvPoint(ru.x, ru.y),20, CV_RGB(255,0,0),3);
   }
  }

 assert(int(corners_2d.size()) == size.width*size.height);

}

/*
 * calibration.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#include "calibration.h"


#include "fstream"

using namespace std;


void update_min_filtered_cloud(Cloud& min_cloud, const Cloud& current){
 for (uint x = 0; x<min_cloud.width; x++)
  for (uint y = 0; y<min_cloud.height; y++){
   pcl_Point old_point = min_cloud.at(x,y);
   pcl_Point new_point = current.at(x,y);

   // if point was nan, update always
   if (!(old_point.x == old_point.x)){
    min_cloud.at(x,y) = new_point;
    continue;
   }

   // update point if new point is valid
   if (!(new_point.x == new_point.x)) continue;

   // z-axis points into sand...
   if (new_point.z > old_point.z)
    min_cloud.at(x,y) = new_point;
  }

}



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

 // ROS_INFO("2d: mean: %f %f, dist: %f", mu[0], mu[1], d);

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

  // ROS_INFO("centered: %f %f", p.x,p.y);

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
 // ROS_INFO("3d: mean: %f %f %f, dist: %f", mu[0], mu[1], mu[2], d);

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

cv::Point2f applyPerspectiveTrafo(const Eigen::Vector3f& p,const cv::Mat& P){
 cv::Point3f p3; p3.x = p.x(); p3.y = p.y(); p3.z = p.z();
 return applyPerspectiveTrafo(p3,P);
}


void applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P, cv::Point2f& p_){
 cv::Mat P4 = cv::Mat(4,1,CV_64FC1);
 cv::Mat P3 = cv::Mat(3,1,CV_64FC1);

 P4.at<double>(0,0) = p.x;
 P4.at<double>(1,0) = p.y;
 P4.at<double>(2,0) = p.z;
 P4.at<double>(3,0) = 1;

 P3 = P*P4;

 double z = P3.at<double>(2);

 p_.x = P3.at<double>(0)/z;
 p_.y = P3.at<double>(1)/z;

}

cv::Point2f applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P){
 cv::Point2f px;
 applyPerspectiveTrafo(p,P,px);
 return px;
}

cv::Point2f applyPerspectiveTrafo(const pcl_Point& p, const cv::Mat& P){
 cv::Point3f p3; p3.x = p.x; p3.y = p.y; p3.z = p.z;
 return applyPerspectiveTrafo(p3,P);
}

void applyHomography(const cv::Point2f& p,const cv::Mat& H, cv::Point2f& p_){

 cv::Mat pc (3,1,CV_64FC1);
 pc.at<double>(0) = p.x;
 pc.at<double>(1) = p.y;
 pc.at<double>(2) = 1;

 pc = H*pc;

 double z = pc.at<double>(2);

 p_.x = pc.at<double>(0)/z;
 p_.y = pc.at<double>(1)/z;
}

void printTrafo(const Eigen::Affine3f& M){
 for (uint i=0;i<4; ++i){
  for (uint j=0;j<4; ++j)
   cout << M(i,j) << " ";
  cout << endl;
 }
}

bool saveAffineTrafo(const Eigen::Affine3f& M, const char* filename){
 ofstream off(filename);
 if (!off.is_open()){ ROS_WARN("Could not write to %s", filename); return false;}
 for (uint i=0;i<4; ++i){
  for (uint j=0;j<4; ++j)
   off << M(i,j) << " ";
  off << endl;
 }

 return true;
}

bool loadAffineTrafo(Eigen::Affine3f& M, const char* filename){
 ifstream iff(filename);

 if (!iff.is_open()) {
  cout << "could not open " << filename << endl;
  return false;
 }

 for (uint i=0;i<4; ++i)
  for (uint j=0;j<4; ++j)
   iff >> M(i,j);

 return true;
}

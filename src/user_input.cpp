/*
 * user_input.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#include "user_input.h"
#include <pcl/PointIndices.h>



using namespace std;


// z points into the wall!!
void User_Input::setCloud(const Cloud& cloud){

 if (cloud.size() == 0) return;

 cloud_ = cloud;

 if (checkerboard_area.size() != 4) return;

 checkerboard_area_3d.clear();
 for (uint i=0; i<checkerboard_area.size(); ++i){
  pcl_Point p = cloud_(checkerboard_area[i].x, checkerboard_area[i].y);
  if (!(p.x == p.x)) return;
  checkerboard_area_3d.push_back(p);
 }


 // get points in front of checkerboard
 pcl::ExtractPolygonalPrismData<pcl_Point> prism_extractor;
 prism_extractor.setInputCloud(cloud_.makeShared());
 prism_extractor.setHeightLimits(-0.03,0.3);
 prism_extractor.setInputPlanarHull(checkerboard_area_3d.makeShared());


 pcl::PointIndices inlier;
 prism_extractor.segment(inlier);

 ROS_INFO("Found %zu inlier", inlier.indices.size());

 prism.clear(); prism.reserve(inlier.indices.size());
 for (uint i=0; i<inlier.indices.size(); ++i){
  pcl_Point p = cloud_[inlier.indices[i]];
  if (!(p.x == p.x)) continue;
  prism.push_back(p);
 }

 Cloud::Ptr msg = prism.makeShared();
 msg->header.frame_id = "/openni_rgb_optical_frame";
 msg->header.stamp = ros::Time::now ();
 pub_filtered.publish(msg);

 processPrismTRIVIAL();

}

// works on the prism data
// or just remove the close points during prism-extraction and then find closes point
void User_Input::processPrismTRIVIAL(){

 finger_found = false;

 float min_dist = -0.05; // everything closer than this is ignored
 fingertip.z = -1e5;
 for (uint i=0; i<prism.size(); ++i){
  pcl_Point p = prism[i];
  if (p.z <= min_dist && p.z > fingertip.z)   fingertip = p;
 }

 if (fingertip.z == -1e5){
  ROS_INFO("No finger");
 }
 else{
  ROS_INFO("Found finger at %f %f %f", fingertip.x, fingertip.y, fingertip.z);
  finger_found = true;
  Cloud foo; foo.push_back(fingertip);
  Cloud::Ptr msg = foo.makeShared();
  msg->header.frame_id = "/openni_rgb_optical_frame";
  msg->header.stamp = ros::Time::now ();
  pub_hand.publish(msg);
 }










}











void projectCloudIntoProjector(const Cloud& cloud, const cv::Mat& P, cv::Mat& img){

 cv::Mat p(4,1,CV_64FC1);
 cv::Mat px(3,1,CV_64FC1);

 int w = img.cols;
 int h = img.rows;

 img.setTo(0);

 float z;

 for (uint i=0; i<cloud.points.size(); ++i){
  p.at<double>(0) = cloud.points[i].x;
  p.at<double>(1) = cloud.points[i].y;
  p.at<double>(2) = cloud.points[i].z;
  p.at<double>(3) = 1;

  z = cloud.points[i].z;

  if (! z == z) continue;

  px = P*p;
  px /= px.at<double>(2);
  int x = px.at<double>(0);	int y = px.at<double>(1);
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


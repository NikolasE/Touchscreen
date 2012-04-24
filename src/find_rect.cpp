/*
 * find_rect.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: engelhan
 */


#include "find_rect.h"


Cloud Rect_finder::find_rect(Cloud& cloud){


 Cloud filtered;


 pcl::PassThrough<pcl_Point> pass;
 pass.setInputCloud (cloud.makeShared());
 pass.setFilterFieldName ("z");
 pass.setFilterLimits (0.5, 1.5);
 //pass.setFilterLimitsNegative (true);
 pass.filter (filtered);

 // pcl::VoxelGrid<pcl_Point> sor;
 // sor.setInputCloud(filtered.makeShared());
 // sor.setLeafSize (0.005f, 0.005f, 0.005f);
 // sor.filter(filtered);


// Cloud result;
//
//  Eigen::Vector4f coefficients;
//
//  std::vector<int> inlier;
//  float quality = fitPlaneToCloud(filtered, coefficients,&inlier);
//
//  ROS_INFO("Cloud had %zu points, %zu inlier", filtered.size(),inlier.size());
//
//
//  for (uint i=0; i<inlier.size(); ++i)
//   result.push_back(filtered.at(inlier.at(i)));


 Cloud::Ptr msg = filtered.makeShared();
 msg->header.frame_id = "/openni_rgb_optical_frame";
 msg->header.stamp = ros::Time::now ();
 pub_rect.publish(msg);



 return filtered;

}

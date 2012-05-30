/*
 * user_input.h
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef USER_INPUT_H_
#define USER_INPUT_H_


#include "calibration.h"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include "projector_calibrator.h"
#include <pcl/segmentation/extract_polygonal_prism_data.h>


void projectCloudIntoProjector(const Cloud& cloud, const cv::Mat& P, cv::Mat& img);



struct User_Input {

 ros::NodeHandle * nh;

 ros::Publisher pub_filtered;
 ros::Publisher pub_hand;

 Projector_Calibrator *calibrator;

 std::vector<cv::Point2i> checkerboard_area;
 Cloud checkerboard_area_3d;


 Cloud cloud_, prism;

 pcl_Point fingertip;
 bool finger_found;

 void setCloud(const Cloud& cloud);

 // TODO: name
 // cloud is in wall-frame
 void processPrismTRIVIAL();

 void init(){
  nh = new ros::NodeHandle();
  finger_found = false;
  pub_filtered = nh->advertise<Cloud>("user_input_full", 1);
  pub_hand = nh->advertise<Cloud>("user_input_hand", 1);
 }


};




#endif /* USER_INPUT_H_ */

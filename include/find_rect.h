/*
 * find_rect.h
 *
 *  Created on: Apr 23, 2012
 *      Author: engelhan
 */

#ifndef FIND_RECT_H_
#define FIND_RECT_H_


#include "cloud_processing.h"
#include "ros/ros.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

struct Rect_finder{

 ros::NodeHandle nh;
 ros::Publisher pub_rect;


 Cloud find_rect(Cloud& cloud);

 Rect_finder(){
  pub_rect = nh.advertise<Cloud>("rectangle", 1);
 }




};




#endif /* FIND_RECT_H_ */

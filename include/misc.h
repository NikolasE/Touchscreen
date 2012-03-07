/*
 * misc.h
 *
 *  Created on: Mar 7, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef MISC_H_
#define MISC_H_


#include "iostream"
#include <opencv/cv.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transform.h>

void saveMatrix(const Eigen::Affine3f& M, const char* filename);
bool loadMatrix(Eigen::Affine3f& M, const char* filename);




#endif /* MISC_H_ */

/*
 * cloud_processing.h
 *
 *  Created on: Feb 28, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef CLOUD_PROCESSING_H_
#define CLOUD_PROCESSING_H_
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_ros/point_cloud.h>

#include <opencv/cv.h>

typedef pcl::Normal Normals;
typedef pcl::PointXYZRGBNormal PointTypeNormal;
typedef pcl::PointXYZRGB pcl_Point;
typedef pcl::PointCloud<pcl_Point> Cloud;



#endif /* CLOUD_PROCESSING_H_ */

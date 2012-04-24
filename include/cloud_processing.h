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
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector2f Vector2f;


using namespace std;


void applyMask(const Cloud& orig, Cloud& masked, const IplImage* mask);

float fitPlaneToCloud(const Cloud& cloud, Eigen::Vector4f &coefficients, std::vector<int>* inliers= NULL);


// returns false if not all points have a depth
bool projectToPlane(const CvPoint2D32f* pts, CvSize size, const Cloud& full_cloud, const Eigen::Vector4f &coefficients, Cloud& projected);


void defineAxis(const Cloud& corners,  Eigen::Vector3f& center, Eigen::Vector3f& upwards, Eigen::Vector3f& right);

void transformInPlaneCoordinates(const Cloud& corners, vector<Vector2f>& coords, const Vector3f& center, const Vector3f& upwards , const Vector3f& right);

void transformInPlaneCoordinates(const CvPoint3D32f p, Vector2f& coords, const Vector3f& center, const Vector3f& upwards , const Vector3f& right);


#endif /* CLOUD_PROCESSING_H_ */

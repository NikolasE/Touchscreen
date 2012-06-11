/*
 * calibration.h
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "type_definitions.h"
#include <opencv/cv.h>

// p_ = H*p
void applyHomography(const cv::Point2f& p,const cv::Mat& H, cv::Point2f& p_);

void applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P, cv::Point2f& p_);
cv::Point2f applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P);

cv::Point2f applyPerspectiveTrafo(const Eigen::Vector3f& p,const cv::Mat& P);


cv::Point2f applyPerspectiveTrafo(const pcl_Point& p, const cv::Mat& P);

void scaleCloud(const Cloud& pts, cv::Mat& U, Cloud& transformed);
void scalePixels(const std::vector<cv::Point2f>& pxs,cv::Mat& T, std::vector<cv::Point2f>& transformed);


bool saveAffineTrafo(const Eigen::Affine3f& M, const char* filename);
bool loadAffineTrafo(Eigen::Affine3f& M, const char* filename);

void printTrafo(const Eigen::Affine3f& M);

void update_min_filtered_cloud(Cloud& min_cloud, const Cloud& current);

#endif /* CALIBRATION_H_ */

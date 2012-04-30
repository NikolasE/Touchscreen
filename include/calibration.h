/*
 * calibration.h
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "cloud_processing.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * Compute Projectionmatrix that projects the points to the given 3d-Coordinates
 * 3d is metric with z~0
 */
void computeHomography_OPENCV(const vector<cv::Point2f>& corners_2d, const Cloud& corners_3d, cv::Mat& H);

void computeHomography_SVD(const vector<cv::Point2f>& corners_2d, const Cloud& corners_3d, cv::Mat& H);




/**
 * draw rectangle into the masked area
 * assumes quite rectangular and axis aligned mask
 * w,h: internal corners (7,7 for chess checkerboard)
 */
void drawCheckerboard(cv::Mat* img,const cv::Mat* mask, cv::Size size, vector<cv::Point2f>& corners_2d);

void projectProjectorIntoImage(const cv::Mat& P, const Eigen::Vector4f& plane_model, const vector<Eigen::Vector3f>& rect, cv::Mat& proj_img);


void drawImageinRect(const cv::Mat& P, const vector<Eigen::Vector3f>& rect, cv::Mat& img);

// p_ = H*p
void applyHomography(const cv::Point2f& p,const cv::Mat& H, cv::Point2f& p_);

void applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P, cv::Point2f& p_);
cv::Point2f applyPerspectiveTrafo(const cv::Point3f& p,const cv::Mat& P);


void scaleCloud(const Cloud& pts, cv::Mat& U, Cloud& transformed);
void scalePixels(const vector<cv::Point2f>& pxs,cv::Mat& T, vector<cv::Point2f>& transformed);



void computeProjectionMatrix(cv::Mat& P, const Cloud& corners, const vector<cv::Point2f>& projector_corners);



#endif /* CALIBRATION_H_ */

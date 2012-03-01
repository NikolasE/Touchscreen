/*
 * calibration.h
 *
 *  Created on: Mar 1, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "cloud_processing.h"

/**
 * Compute Projectionmatrix that projects the points to the given 3d-Coordinates
 * 3d is metric with z~0
 */
void computeHomography(const vector<CvPoint2D32f>& corners_2d, const Cloud& corners_3d, CvMat* H);

/**
 * draw rectangle into the masked area
 * assumes quite rectangular and axis aligned mask
 * w,h: internal corners (7,7 for chess checkerboard)
 */
void drawCheckerboard(IplImage* img,const IplImage* mask, int w, int h, vector<CvPoint2D32f>& corners_2d);




#endif /* CALIBRATION_H_ */

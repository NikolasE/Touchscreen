/*
 * cloud_processing.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: Nikolas Engelhard
 */


#include "cloud_processing.h"


// apply mask to orig (remove all points with mask(p)==0 and p.x = NAN)
void applyMask(const Cloud& orig, Cloud& masked, const IplImage* mask){
	assert(mask);
	masked.points.clear();
	//	cout << "full cloud has " << orig.points.size() << endl;
	for (int x=0; x<mask->width; ++x)
		for (int y=0; y<mask->height; ++y){
			if (cvGet2D(mask,y,x).val[0] == 0) continue;
			Point p = orig.at(x,y);
			if (p.x == p.x){ // cloud contains NANs!
				masked.points.push_back(p);
//				cout << p.x << endl;
			}
		}
	//	cout << "filtered cloud has " << masked.points.size() << endl;
}


// fit plane to cloud, returns percentage of inliers
float fitPlaneToCloud(const Cloud& cloud, Eigen::Vector4f &coefficients){
	// http://pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus
  pcl::SampleConsensusModelPlane<Point>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<Point> (cloud.makeShared()));

	float max_dist_m = 0.01;

	std::vector<int> inliers;
	pcl::RandomSampleConsensus<Point> ransac (model_p);
	ransac.setDistanceThreshold(max_dist_m);
	ransac.computeModel();
	ransac.getInliers(inliers);

	Eigen::VectorXf c;
	ransac.getModelCoefficients(c);
	coefficients = c;

	float inlier_pct = inliers.size()*100.0/cloud.points.size();

	if (inlier_pct<0.5){ ROS_WARN("Only %.3f %%  inlier in fitPlaneToCloud!", inlier_pct); }
	return inlier_pct;

}

bool projectToPlane(const CvPoint2D32f* pts, CvSize size, const Cloud& full_cloud, const Eigen::Vector4f &coefficients, Cloud& projected){

	// create new Pointcloud from corners:
	Cloud corners;
	int pnt_cnt = size.width*size.height;
	corners.points.resize(pnt_cnt);
	corners.width = size.width;
	corners.height = size.height;

	vector<int> inlier; inlier.resize(pnt_cnt);
	for (int i=0; i<pnt_cnt; ++i){
		Point p = full_cloud.at(pts[i].x, pts[i].y);
		if (!(p.x == p.x)){ROS_WARN("projectToPlane: Found Corner without depth!"); return false; }
		corners.points.push_back(p);
//		cout << "p: " << p.x << endl;
		inlier.push_back(i);// add every point as inlier
	}

	pcl::SampleConsensusModelPlane<Point>::Ptr
	    model (new pcl::SampleConsensusModelPlane<Point> (corners.makeShared()));
	model->projectPoints(inlier, coefficients,projected);




	return true;
}


void defineAxis(const Cloud& corners, Eigen::Vector3f& center, Eigen::Vector3f& upwards, Eigen::Vector3f& right){

	Point c  = corners.at(corners.width/2, corners.height/2);
	Point up = corners.at(corners.width/2, 0);
	Point r  = corners.at(corners.width-1, corners.height/2);

	center = Eigen::Vector3f(c.x,c.y,c.z);
	upwards = Eigen::Vector3f(up.x,up.y,up.z)-center;
	right = Eigen::Vector3f(r.x,r.y,r.z)-center;

}

// all points are in one plane!
void transformInPlaneCoordinates(const Cloud& corners, vector<Vector2f>& coords, const Vector3f& center, const Vector3f& upwards , const Vector3f& right){

	coords.resize(corners.points.size());
	for (uint i=0; i<corners.points.size(); ++i){
		Point c = corners.points[i];
		Vector3f p = Eigen::Vector3f(c.x,c.y,c.z)-center;

		float u = upwards.dot(p);
		float r = right.dot(p);

		//if (u>1.1 || r>1.1){
			ROS_WARN("u,r: %f %f", u,r);
//		}

	}


}




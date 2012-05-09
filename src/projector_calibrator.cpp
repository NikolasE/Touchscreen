/*
 * projector_calibrator.cpp
 *
 *  Created on: May 9, 2012
 *      Author: engelhan
 */

#include "projector_calibrator.h"




void Projector_Calibrator::computeHomography_OPENCV(const vector<cv::Point2f>& projector_corners){

 ROS_WARN("COMPUTING HOMOGRAPHY WITH openCV");

 uint N = projector_corners.size();
 if(observations_3d.size() < N){
  ROS_ERROR("computeHomography_OPENCV: less 3d-points than 2d-points, aborting");
  return;
 }
 // count number of 3d points which are more than 2cm away from the z=0-plane
 float z_max = 0.03; int cnt = 0;
 for (uint i=0; i<N; ++i) { if (abs(observations_3d.at(i).z) > z_max) cnt++; }
 if (cnt>N*0.01) {  ROS_WARN("found %i of %i points with dist > 3cm", cnt,N); }


 // copy the 3d observations (x,y,~0) into 2d (x,y)
 vector<cv::Point2f> src; src.reserve(N);
 for (uint i=0; i<N; ++i){src.push_back(cv::Point2f(observations_3d.at(i).x,observations_3d.at(i).y));}

 // 2d = H*3d H*(x,y,1)

#define DO_SCALING

#ifdef DO_SCALING
 cv::Mat T,U;
 vector<cv::Point2f> src_trafoed, d2_trafoed;
 scalePixels(src,  T, src_trafoed);
 scalePixels(projector_corners,  U, d2_trafoed);
 hom_CV = cv::findHomography(src_trafoed,d2_trafoed);
 hom_CV = U.inv()*hom_CV*T;
#else
 hom_CV = cv::findHomography(src,projector_corners);
#endif


 cout << "Homography with OpenCV: " << endl << hom_CV << endl;

 // compute error:
 float error = 0;

 cv::Point2f px;
 float err_x = 0;
 float err_y = 0;

 float e_x, e_y;

 for (uint i=0; i<N; ++i){
  applyHomography(cv::Point2f(observations_3d.at(i).x,observations_3d.at(i).y),hom_CV, px);

  e_x = abs((px.x-projector_corners.at(i).x));
  e_y = abs((px.y-projector_corners.at(i).y));

  err_x += e_x/N; err_y += e_y/N  ;
  error += sqrt(e_x*e_x+e_y*e_y)/N;

  //  ROS_INFO("Proj: %f %f, goal: %f %f (Error: %f)", px.x,px.y,corners_2d.at(i).x, corners_2d.at(i).y,err);
 }


 ROS_INFO("mean error: %f (x: %f, y: %f)", error, err_x, err_y);

 // cout << "write" << endl << "homography_OPENCV" << endl;
 //  cv::FileStorage fs("data/homography_OPENCV.yml", cv::FileStorage::WRITE);
 //  assert(fs.isOpened());
 //  fs << "ProjectionMatrix" << hom_CV;
 //  fs.release();


}


void Projector_Calibrator::computeHomography_SVD(const vector<cv::Point2f>& projector_corners){

 ROS_WARN("COMPUTING HOMOGRAPHY WITH SVD");

 uint N = projector_corners.size();
 if(observations_3d.size() < N){
  ROS_ERROR("computeHomography_SVD: less 3d-points than 2d-points, aborting");
  return;
 }

 // count number of 3d points which are more than 2cm away from the z=0-plane
 float z_max = 0.03; int cnt = 0;
 for (uint i=0; i<N; ++i) { if (abs(observations_3d.at(i).z) > z_max) cnt++; }
 if (cnt>N*0.01) { ROS_WARN("found %i of %i points with dist > 3cm", cnt,N); }

 // create Matrices
 // cv::Mat src(2,N,CV_32FC1);
 // cv::Mat dst(2,N,CV_32FC1);

#define SCALE_SVD


#ifdef SCALE_SVD
 cv::Mat T;
 vector<cv::Point2f> d2_trafoed;
 scalePixels(projector_corners,  T, d2_trafoed);
#else
 vector<cv::Point2f> d2_trafoed = projector_corners;
#endif



 // Pointcloud to 2d-points
 vector<cv::Point2f> src, src_trafoed;

 for (uint i=0; i<N; ++i) src.push_back(cv::Point2f(observations_3d.at(i).x,observations_3d.at(i).y));

#ifdef SCALE_SVD
 cv::Mat U;
 scalePixels(src, U,src_trafoed);
#else
 src_trafoed = src;
#endif


 cv::Mat A = cv::Mat(2*N,9,CV_64FC1);
 A.setTo(0);

 // p_ cross H*p = 0
 for (uint i=0; i<N; ++i){
  cv::Point2f P = src_trafoed.at(i);
  cv::Point2f p = d2_trafoed.at(i);

  // ROS_INFO("P: %f %f,  p: %f %f", P.x, P.y, p.x, p.y);

  float f[9] = {0,0,0,-P.x,-P.y,1,p.y*P.x,p.y*P.y,p.y};
  for (uint j=0; j<9; ++j) A.at<double>(2*i,j) = f[j];

  float g[9] = {P.x,P.y,1,0,0,0,-p.x*P.x,-p.x*P.y,-p.x};
  for (uint j=0; j<9; ++j) A.at<double>(2*i+1,j) = g[j];
 }
 // now solve A*h == 0
 // Solution is the singular vector with smallest singular value

 cv::Mat h = cv::Mat(9,1,CV_64FC1);
 cv::SVD::solveZ(A,h);


 // h only fixed up to scale -> set h(3,3) = 1;
 h /= h.at<double>(8);

// cout << "h: " << h << endl;

 hom_SVD = cv::Mat(3,3,CV_64FC1);

 for (uint i=0; i<3; ++i){
  for (uint j=0; j<3; ++j)
   hom_SVD.at<double>(i,j) =  h.at<double>(3*i+j);
 }

 // cout << "Tinv " << T.inv() << endl;
 // cout << "U " << U << endl;

 // undo scaling
#ifdef SCALE_SVD
 hom_SVD = T.inv()*hom_SVD*U;
#endif

 // 2d = H*3d H*(x,y,1)


 cout << "Homography with SVD: " << endl << hom_SVD << endl;


 // compute error:
 float error = 0;
 float err_x = 0;
 float err_y = 0;

 float a,b;


 cv::Point2f px;
 for (uint i=0; i<N; ++i){
  applyHomography(src.at(i), hom_SVD, px);

  a = abs(px.x-projector_corners.at(i).x);
  b = abs(px.y-projector_corners.at(i).y);

  err_x += a/N; err_y += b/N;

  error += sqrt(a*a+b*b)/N;

  //ROS_INFO("src: %f %f, Proj: %f %f, goal: %f %f (Error: %f)", src.at(i).x, src.at(i).y, px.x,px.y,corners_2d.at(i).x, corners_2d.at(i).y,err);
 }

 ROS_INFO("mean error: %f (x: %f, y: %f)", error, err_x, err_y);


 // cout << "write" << endl << "homography_SVD" << endl;
 //  cv::FileStorage fs("data/homography_SVD.yml", cv::FileStorage::WRITE);
 //  assert(fs.isOpened());
 //  fs << "ProjectionMatrix" << hom_SVD;
 //  fs.release();


}

void Projector_Calibrator::computeProjectionMatrix(const vector<cv::Point2f>& projector_corners){

  uint c_cnt = observations_3d.points.size();
  uint proj_cnt = projector_corners.size();

  assert(c_cnt >0 && c_cnt % proj_cnt == 0);

  int img_cnt = c_cnt/proj_cnt;

  ROS_WARN("Computing Projection matrix from %i images", img_cnt);


  Cloud trafoed_corners;
  vector<cv::Point2f> trafoed_px;
  cv::Mat U,T;

  scaleCloud(observations_3d, U, trafoed_corners);
  scalePixels(projector_corners, T, trafoed_px);

  cv::Mat A = cv::Mat(2*c_cnt,12,CV_64FC1);
  A.setTo(0);

  //ROS_ERROR("Projection:");

  // p_ cross H*p = 0
  for (uint i=0; i<c_cnt; ++i){
   pcl_Point   P = trafoed_corners.points.at(i);
   cv::Point2f p = trafoed_px.at(i%proj_cnt);

  //  ROS_INFO("from %f %f %f to %f %f", P.x,P.y,P.z,p.x,p.y);


   float f[12] = {0,0,0,0,-P.x,-P.y,-P.z,1,p.y*P.x,p.y*P.y,p.y*P.z,p.y};
   for (uint j=0; j<12; ++j) A.at<double>(2*i,j) = f[j];

   float g[12] = {P.x,P.y,P.z,1,0,0,0,0,-p.x*P.x,-p.x*P.y,-p.x*P.z,-p.x};
   for (uint j=0; j<12; ++j) A.at<double>(2*i+1,j) = g[j];
  }

  // now solve A*h == 0
  // Solution is the singular vector with smallest singular value

  cv::Mat h = cv::Mat(12,1,CV_64FC1);
  cv::SVD::solveZ(A,h);

  proj_matrix = cv::Mat(3,4,CV_64FC1);

  for (uint i=0; i<3; ++i){
   for (uint j=0; j<4; ++j)
    proj_matrix.at<double>(i,j) =  h.at<double>(4*i+j);
  }

  // cout << "Tinv " << T.inv() << endl;
  // cout << "U " << U << endl;

  // undo scaling
  proj_matrix = T.inv()*proj_matrix*U;


 // cout << "Projection Matrix: " << endl <<  P << endl;


  // compute reprojection error:
  double total = 0;
  double total_x = 0;
  double total_y = 0;

  cv::Point2f px;



  for (uint i=0; i<c_cnt; ++i){
   //    ROS_INFO("projection %i", i);

   pcl_Point   p = observations_3d.points.at(i);
   cv::Point2f p_ = projector_corners.at(i%proj_cnt); //

   applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),proj_matrix,px);


   // ROS_INFO("err: %f %f", (px.x-p_.x),(px.y-p_.y));

   total_x += abs(px.x-p_.x)/c_cnt;
   total_y += abs(px.y-p_.y)/c_cnt;
   total += sqrt(pow(px.x-p_.x,2)+pow(px.y-p_.y,2))/c_cnt;

  }

  ROS_INFO("Projection Matrix: mean error: %f (x: %f, y: %f)", total, total_x, total_y);


  // cout << "write" << endl << proj_Matrix << endl;
  //  cv::FileStorage fs("data/projection_matrix.yml", cv::FileStorage::WRITE);
  //  assert(fs.isOpened());
  //  fs << "ProjectionMatrix" << proj_Matrix;
  //  fs.release();



}





bool Projector_Calibrator::findCheckerboardCorners(){
 corners.clear();
 if (input_image.rows == 0){  ROS_WARN("can't find Corners on empty image!"); return false;  }
 if (!cv::findChessboardCorners(input_image, C_checkboard_size,corners, CV_CALIB_CB_ADAPTIVE_THRESH)) return false;
 cv::cvtColor(input_image, gray, CV_BGR2GRAY);
 cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
 return true;
}


bool Projector_Calibrator::storeCurrent3DObservations(){

 if (corners.size() == 0){ ROS_WARN("Can't add Observations since Corners havn't been detected"); return false;}
 assert(input_cloud.size() > 0);
 Cloud c_3d;
 for (uint i=0; i<corners.size(); ++i){
   pcl_Point p = input_cloud.at(corners[i].x, corners[i].y);
   if (!(p.x == p.x)){ROS_WARN("storeCurrent3DObservations: Found Corner without depth!"); return false;}
   c_3d.points.push_back(p);
  }
 // transform from kinect-frame to wall-frame
 pcl::getTransformedPointCloud(c_3d, kinect_trafo, c_3d);
 observations_3d.points.insert(observations_3d.points.end(),c_3d.points.begin(), c_3d.points.end());
 ROS_INFO("Added %zu points, now %zu 3D-Observations",c_3d.size(), observations_3d.size());
 return true;
}



void Projector_Calibrator::computeKinectTransformation(){

 if (!kinect_orientation_valid){
  ROS_INFO("Can't compute KinectTrafo without Kinect's orientation angle"); return;
 }

 Cloud filtered;
 applyMaskOnInputCloud(filtered);

 Eigen::Vector4f plane_model;
 fitPlaneToCloud(filtered, plane_model);

 int m = (C_checkboard_size.height/2*C_checkboard_size.width)+(C_checkboard_size.width-1)/2;

 pcl_Point p  = input_cloud.at(corners[m].x, corners[m].y);
 pcl_Point p2 = input_cloud.at(corners[m].x+sin(-kinect_tilt_angle_deg/180*M_PI)*100, corners[m].y-cos(-kinect_tilt_angle_deg/180*M_PI)*100);

 if ( p2.x != p2.x){
  ROS_WARN("NAN in pointcloud, no calculation of new wall-system");
  return;
 }

 Eigen::Vector3f pl_center = Eigen::Vector3f(p.x,p.y,p.z);
 Eigen::Vector3f pl_upwards = Eigen::Vector3f(p2.x-p.x,p2.y-p.y,p2.z-p.z);

 float plane_direction = 1;
 if (plane_model.head<3>()[2] < 0){ plane_direction  = -1; }

 pcl::getTransformationFromTwoUnitVectorsAndOrigin(-pl_upwards,plane_direction*plane_model.head<3>(), pl_center, kinect_trafo);

 //  saveMatrix(kinect_trafo,"data/kinect_trafo.txt");
 //  ROS_INFO("Wrote kinect_trafo to data/kinect_trafo.txt");

 kinect_trafo_valid = true;

}


// TODO: assumption: only rectangular mask...
void Projector_Calibrator::setMaskFromDetections(){

 if (corners.size() != uint(C_checkboard_size.width*C_checkboard_size.height)){
  ROS_INFO("can't create mask if the corners were not detected!"); return; }

 mask = cv::Mat(cv::Size(480,640), CV_8UC1);  mask.setTo(0);

 int w = C_checkboard_size.width; int h = C_checkboard_size.height;
 int l = corners[1].x-corners[0].x; // length of a square
 float min_x = corners[0].x-l;    float min_y = corners[0].y-l;
 float max_x = corners[w*h-1].x+l; float max_y = corners[w*h-1].y+l;
 cv::rectangle(mask, cv::Point(min_x,min_y), cv::Point(max_x,max_y), CV_RGB(255,0,0),-1);

}



float Projector_Calibrator::fitPlaneToCloud(const Cloud& cloud, Eigen::Vector4f& model){
 ROS_INFO("Fitting plane to cloud with %zu points", cloud.size());

 // http://pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus
 pcl::SampleConsensusModelPlane<pcl_Point>::Ptr
 model_p (new pcl::SampleConsensusModelPlane<pcl_Point> (cloud.makeShared()));

 pcl::RandomSampleConsensus<pcl_Point> ransac(model_p);
 ransac.setDistanceThreshold(0.005); // max dist of 5mm
 ransac.computeModel();

 Eigen::VectorXf c;
 ransac.getModelCoefficients(c);
 model = c;

 std::vector<int> inliers;
 ransac.getInliers(inliers);
 float inlier_pct = inliers.size()*100.0/cloud.size();

 if (inlier_pct<0.5){ ROS_WARN("Only %.3f %%  inlier in fitPlaneToCloud!", inlier_pct); }
 return inlier_pct;
}


void Projector_Calibrator::applyMaskOnInputCloud(Cloud& out){
 assert(mask_valid());
 for (int x=0; x<mask.rows; ++x)
  for (int y=0; y<mask.cols; ++y){
   if (mask.at<uchar>(y,x) > 0){
    pcl_Point p = input_cloud.at(x,y);
    if (p.x == p.x) out.points.push_back(p);
   }
  }
}



/*
 * meshing.h
 *
 *  Created on: Apr 20, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef MESHING_H_
#define MESHING_H_

#include "cloud_processing.h"
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/io/vtk_io.h>
#include "visualization_msgs/MarkerArray.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_greedy.h>
#include <pcl/features/normal_3d.h>


pcl::PolygonMesh createMeshFromPointcloud(const Cloud& cloud);



/*
 * Create mesh from organized pointcloud
 */
pcl::PolygonMesh createSimpleMeshFromPointcloud(const Cloud& cloud, float max_dist_cm);



struct Mesh_visualizer {

 ros::NodeHandle nh_;
 ros::Publisher pub_, pub_lines_;

 Mesh_visualizer(){
  pub_ = nh_.advertise<visualization_msgs::Marker>( "mesh", 0 );
  pub_lines_ = nh_.advertise<visualization_msgs::Marker>( "mesh_lines", 0 );
 }

// TODO: cloud should somehow already included in the mesh...
 void visualizeMesh(const Cloud& cloud, const pcl::PolygonMesh& mesh);
 void visualizeMeshLines(const Cloud& cloud, const pcl::PolygonMesh& mesh);






};



#endif /* MESHING_H_ */

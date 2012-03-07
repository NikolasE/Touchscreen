/*
 * misc.cpp
 *
 *  Created on: Mar 7, 2012
 *      Author: Nikolas Engelhard
 *
 */

#include "misc.h"
#include "fstream"

using namespace std;

void saveMatrix(const Eigen::Affine3f& M, const char* filename){
	ofstream off(filename);
	assert(off.is_open());
	for (uint i=0;i<4; ++i){
		for (uint j=0;j<4; ++j)
			off << M(i,j) << " ";
		off << endl;
	}
}

bool loadMatrix(Eigen::Affine3f& M, const char* filename){
	ifstream iff(filename);

	if (!iff.is_open()) {
		cout << "could not open " << filename << endl;
		return false;
	}

	for (uint i=0;i<4; ++i)
		for (uint j=0;j<4; ++j)
			 iff >> M(i,j);

	return true;
}


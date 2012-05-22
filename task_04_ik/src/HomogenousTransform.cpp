/*
 * HomogenousTransform.cpp
 *
 *  Created on: May 21, 2012
 *      Author: matthias
 */

#include "HomogenousTransform.h"

#include <iostream>

HomogenousTransform ht_from_rpy(double roll, double pitch, double yaw) {
	HomogenousTransform ht;

	ht = Eigen::AngleAxis<double>(roll, Eigen::Vector3d(1,0,0))
		* Eigen::AngleAxis<double>(pitch, Eigen::Vector3d(0,1,0))
		* Eigen::AngleAxis<double>(yaw, Eigen::Vector3d(0,0,1));

	//std::cout << "ht_from_rpy: " << std::endl<< ht.affine() << std::endl;

	return ht;

}

HomogenousTransform ht_from_xyz(double x, double y, double z) {
	HomogenousTransform ht;

	ht = Eigen::Translation<double,3>(x,y,z);

	//	std::cout << "ht_from_xyz: " << std::endl<< ht.affine() << std::endl;

	return ht;
}

HomogenousTransform ht_from_xyzrpy(double x, double y, double z, double roll, double pitch, double yaw) {
	return ht_from_rpy(roll, pitch, yaw) * ht_from_xyz(x,y,z);
}

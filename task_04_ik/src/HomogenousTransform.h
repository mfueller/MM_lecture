/*
 * HomogenousTransform.h
 *
 *  Created on: May 21, 2012
 *      Author: matthias
 */

#ifndef HOMOGENOUSTRANSFORM_H_
#define HOMOGENOUSTRANSFORM_H_

#include <Eigen/Dense>
#include <math.h>

typedef Eigen::Matrix<double, 1, 4> DH_Parameter;
typedef Eigen::Matrix<double, Eigen::Dynamic, 4>        DH_Parameters;

typedef Eigen::Matrix<double, 1, Eigen::Dynamic>        JointParameter;

typedef Eigen::Transform<double, 3, Eigen::Affine> 	HomogenousTransform;
typedef Eigen::Matrix<double, 3, 3> 			RotationMatrix;
typedef HomogenousTransform::TranslationPart		TranslationVector;


#define HT_R_X 0
#define HT_R_Y 1
#define HT_R_Z 2

#define HT_T_X 0
#define HT_T_Y 1
#define HT_T_Z 2

#define HT_BLOCK_RX 0,0,3,1
#define HT_BLOCK_RY 0,1,3,1
#define HT_BLOCK_RZ 0,2,3,1

HomogenousTransform ht_from_rpy(double roll, double pitch, double yaw);

HomogenousTransform ht_from_xyz(double x, double y, double z);

HomogenousTransform ht_from_xyzrpy(double x, double y, double z, double roll, double pitch, double yaw);


#endif /* HOMOGENOUSTRANSFORM_H_ */

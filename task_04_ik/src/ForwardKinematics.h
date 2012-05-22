/*
 * ForwardKinematics.h
 *
 *  Created on: May 21, 2012
 *      Author: matthias
 */

#ifndef FORWARDKINEMATICS_H_
#define FORWARDKINEMATICS_H_


#include "HomogenousTransform.h"

#include <vector>

typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Jacobian;


class ForwardKinematics {
public:
	ForwardKinematics(DH_Parameters parameter);
	ForwardKinematics();
	virtual ~ForwardKinematics();

	HomogenousTransform calculateForwardKinematics(JointParameter parameter, int origin_frame=0, int target_frame=-1);

	HomogenousTransform getTransform(int dh_line, double theta);

	std::vector<HomogenousTransform> getSingleTransformations(JointParameter parameter);

	Jacobian getJacobian(JointParameter q);


protected:


	DH_Parameters dh_table;
};

#endif /* FORWARDKINEMATICS_H_ */

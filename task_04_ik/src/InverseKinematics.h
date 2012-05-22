#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#include <vector>
#include "HomogenousTransform.h"

class InverseKinematics {
public:

	InverseKinematics();
	virtual ~InverseKinematics();


	virtual bool solve(
			double target_x, double target_y, double target_z,
			double target_roll, double target_pitch, double target_yaw,
			JointParameter& currentConfig,
			std::vector<JointParameter>& path) = 0;

};

#endif /* INVERSEKINEMATICS_H_ */

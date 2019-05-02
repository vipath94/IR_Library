#ifndef FORWARDKINEMATICS_HPP
#define FORWARDKINEMATICS_HPP

#include <vector>
#include "TypeDefs.hpp"
#include "RigidBodyMotion.hpp"

namespace IRlibrary
{
	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Blist expressed in the end-effector frame, and the list of joint values thetalist **/
	SE3Mat FKinBody (SE3Mat M, std::vector <ScrewAxis> Blist, std::vector <double> thetaList){
		SE3Mat T = M;
		for (int i = 0; i < int(thetaList.size()); i++) {
			T = T * MatrixExp6(VecTose3(Blist[i]*thetaList[i]));
		}
                return T;
	}

	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Slist expressed in the space frame, and the list of joint values thetalist **/
	SE3Mat FKinSpace (SE3Mat M, std::vector <ScrewAxis> Slist, std::vector <double> thetaList){
		SE3Mat T = M;
		for (int i = int(thetaList.size() - 1); i > -1; i--) {
			T = MatrixExp6(VecTose3(Slist[i]*thetaList[i])) * T;
		}
                return T;
	}

} /* IRlibrary */ 

#endif /* ifndef FORWARDKINEMATICS_HPP */

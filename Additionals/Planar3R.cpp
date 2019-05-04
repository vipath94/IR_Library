#include "Planar3R.hpp"

namespace IRlibrary
{
	void Planar3R::zeroForwardKinematics() 
        {
		x[0] = base[0] + l[0] * cos(q[0])+ l[1] * cos(q[0]+q[1]) + l[2] * cos(q[0]+q[1]+q[2]);
		x[1] = base[1] + l[0] * sin(q[0])+ l[1] * sin(q[0]+q[1]) + l[2] * sin(q[0]+q[1]+q[2]);
		x[2] = q[0] + q[1] + q[2];
	}
        
        bool Planar3R::inverseKinematics_numerical(Eigen::VectorXd const &init_thetalist,Vec3 const &x_in)
        {
            SE3Mat T;
            double ctht = cos(x_in[2]);double stht = sin(x_in[2]);
            T << ctht, -stht, 0, x_in[0],
                 stht, ctht, 0, x_in[1],
                 0,0,1,0,
                 0,0,0,1;

            Eigen::VectorXd thetalist;
            if (IKinSpace(Slist, M, T, init_thetalist, thetalist))
            {
                std::cout<<"Planar3R::inverseKinematics_numerical [Error] failed to converge or at singularity.\n";
                return 1;
            }
            x = x_in;
            q << thetalist;
            return 0;
        }
 
        bool Planar3R::checkInDexWs(Vec3 x_in)
        {
            double pDist = (x_in.head(2)-base).norm();
            if (pDist > wsRad[2] or pDist < wsRad[1])
                return false;
            else
                return true;
        }           
	
} /* IRlibrary */ 



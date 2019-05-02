#ifndef JACOBIAN
#define JACOBIAN

#include "RigidBodyMotion.hpp"
#include "TypeDefs.hpp"
#include <vector>
#include <cmath>

namespace IRlibrary {

    JacobianMat JacobianSpace(std::vector <ScrewAxis> s_list, std::vector <double>  theta_list) 
    {
        size_t n  = s_list.size();
        int m = theta_list.size();

	JacobianMat Js(6,n);
        Js.col(0) = s_list[0];
	SE3Mat T;
        T << 1,0,0,0,
             0,1,0,0,
             0,0,1,0,
             0,0,0,1;
 
	Twist tmp(s_list[0].size());

	for (int i = 1; i < m; i++) 
        {
	    tmp << s_list[i - 1] * theta_list[i - 1];
	    T = T * MatrixExp6(VecTose3(tmp));
	    
	    Js.col(i) = Adjoint(T) * s_list[i];
	}

	return Js;

	}

    JacobianMat JacobianBody( std::vector <ScrewAxis> b_list, std::vector <double>  theta_list) 
    {
        size_t n  = b_list.size();
        int m = theta_list.size();
        
	JacobianMat Jb(6,n);
        Jb.col(0) = b_list[0];

	SE3Mat T;
        T << 1,0,0,0,
             0,1,0,0,
             0,0,1,0,
             0,0,0,1;

	Twist tmp(b_list[0].size());

	for (int i = m - 2; i >= 0; i--) 
        {
	    tmp << b_list[i + 1] * theta_list[i + 1];
	    T = T * MatrixExp6(VecTose3(-1 * tmp));
	    
	    Jb.col(i) = Adjoint(T) * b_list[i];
	}
        
	return Jb;
  
    }
	
    


} /* IRlibrary */
#endif/*ifdef JACOBIAN*/

#include <RigidBodyMotion.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "TypeDefs.hpp"
#include "MathUtils.hpp"

namespace IRlibrary {


	/** Returns true if a 3X3 matrix satisfies properties of SO(3) **/
	bool isSO3(SO3Mat mat){
            SO3Mat mat_t = mat.transpose();
            SO3Mat mat_r = mat_t * mat;
            if (mat_r.isIdentity() && (mat.determinant() == 1) )
                return true;
            else
                return false;
	}

	/** Returns true if a 3x3 matrix is skew-symmetric **/
	bool isso3(so3Mat mat){
            so3Mat mat_s = -1 * mat.transpose();
            if (mat == mat_s)
                return true;
            else
                return false;
	}

	/** Returns true if a 4x4 matrix is SE(3) **/
	bool isSE3(SE3Mat mat){
            SO3Mat mat_r;
            mat_r << mat(0,0), mat(0,1), mat(0,2),
                     mat(1,0), mat(1,1), mat(1,2),
                     mat(2,0), mat(2,1), mat(2,2);  
       
            if (isSO3(mat_r) && mat(3,0) == 0 && mat(3,1) == 0 && mat(3,2) == 0 && mat(3,3) == 1)
                return true;
            else
                return false;
	}


	/** Returns true if a 4x4 matrix is se(3) **/
	bool isse3(se3Mat mat){
            so3Mat mat_s;
            mat_s << mat(0,0), mat(0,1), mat(0,2),
                     mat(1,0), mat(1,1), mat(1,2),
                     mat(2,0), mat(2,1), mat(2,2);         
            if (isso3(mat_s) && mat(3,0) == 0 && mat(3,1) == 0 && mat(3,2) == 0 && mat(3,3) == 0)
                return true;
            else
                return false;
	}

	/** Checks if the vector is unit **/
	bool isUnit(Vec2 vec){
            if(vec.norm() == 1)
	        return true;
            else 
                return false;
	}
	bool isUnit(Vec3 vec){
            if(vec.norm() == 1)
	        return true;
            else 
                return false;
	}
	bool isUnit(Vec4 vec){
            if(vec.norm() == 1)
	        return true;
            else 
                return false;
	}


	/** Computes the inverse of rotation matrix **/
	SO3Mat RotInv(SO3Mat mat){
            SO3Mat mat_t = mat.transpose();    
	    return mat_t;
	}

	/** Computes the 3X3 skew-symmetric matrix corresponding to 3x1 omega vector **/
	so3Mat VecToso3(Vec3 omega){
	    so3Mat mat;
	    mat << 0, -omega(2), omega(1),
                   omega(2), 0, -omega(0), 
                   -omega(1), omega(0), 0;
	    return mat;
	}

	/** Computes the 3-vector corresponding to 3x3 skew-symmetric matrix **/
	Vec3 so3ToVec(so3Mat mat){
	    Vec3 omega;
            omega(0) = mat(2,1);
            omega(1) = mat(0,2);
            omega(2) = mat(1,0);
               
	    return omega;
	}

	/** Extracts the rotation axis, omega_hat, and the rotation amount, theta, from the 3-vector omega_hat theta of exponential coordinates for rotation, expc3 **/
	AxisAngle AxisAng3 (Vec3 expc3) {
		AxisAngle axisAngle;
		axisAngle.theta = expc3.norm();
		axisAngle.omega = expc3.normalized();
		return axisAngle;
	}

	/** Computes the rotation matrix R \in SO(3) corresponding to the matrix exponential of so3Mat **/
	SO3Mat MatrixExp3 (so3Mat in_mat){
		SO3Mat mat;
		mat << 1, 0, 0,
                       0, 1, 0,
		       0, 0, 1;
                Vec3 omega = so3ToVec(in_mat);
                
                AxisAngle angle = (AxisAng3(omega));
		so3Mat mat_a = VecToso3(angle.omega);
		return mat + (sin(angle.theta) * mat_a) + ((1 - cos(angle.theta)) * (mat_a * mat_a)); 
	}

	/** Computes the matrix logarithm so3mat \in so(3) of the rotation matrix R \in SO(3) **/
	so3Mat MatrixLog3(SO3Mat in_mat){
                double acosinput = (in_mat.trace() - 1) / 2.0;
		so3Mat m_ret;
                m_ret << 0,0,0,
                         0,0,0,
                         0,0,0;
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Vec3 omg;
                        Vec3 tmp;
			if (!nearZero(1 + in_mat(2, 2)))
                        {
                                tmp << in_mat(0, 2), in_mat(1, 2), (1 + in_mat(2, 2));
				omg = (1.0 / sqrt(2 * (1 + in_mat(2, 2))))*tmp;
                        }
			else if (!nearZero(1 + in_mat(1, 1)))
                        {
                                tmp << in_mat(0, 1), (1 + in_mat(1, 1)), in_mat(2, 1);
				omg = (1.0 / sqrt(2 * (1 + in_mat(1, 1))))*tmp;
                        }
			else
                        {
                                tmp << (1 + in_mat(0, 0)), in_mat(1, 0), in_mat(2, 0);
				omg = (1.0 / sqrt(2 * (1 + in_mat(0, 0))))*tmp;
                        }
			m_ret = VecToso3(M_PI * omg);
			return m_ret;
		}
		else {
			double theta = acos(acosinput);
			m_ret = theta / 2.0 / sin(theta)*(in_mat - in_mat.transpose());
			return m_ret;
		}
	 }

	/** Compute the 4x4 transformation matrix **/
	SE3Mat RpToTrans(SO3Mat R, Vec3 p){
		SE3Mat T;
		T << R(0,0),R(0,1) , R(0,2), p(0),
		     R(1,0),R(1,1) , R(1,2), p(1),
		     R(2,0),R(2,1) , R(2,2), p(2),
		     0, 0, 0, 1;
		return T;
	}

	/** Extracts the rotation matrix and position vector from homogeneous transformation matrix **/
	void TransToRp(SE3Mat mat, SO3Mat &R, Vec3 &p){
		R = mat.block<3, 3>(0, 0);
		p << mat(0,3), mat(1,3), mat(2,3);
	}

	/** Inverse of transformation matrix **/
	SE3Mat TransInv(SE3Mat mat){
                SO3Mat R;
                Vec3 p;
                TransToRp(mat,R,p);
                SO3Mat R_Trans = R.transpose();
                Vec3 R_snd = -R_Trans * p;
                SE3Mat mat_i;
                mat_i << R_Trans(0,0), R_Trans(0,1), R_Trans(0,2),  R_snd(0),
                         R_Trans(1,0), R_Trans(1,1), R_Trans(1,2),  R_snd(1),
                         R_Trans(2,0), R_Trans(2,1), R_Trans(2,2),  R_snd(2),
                         0,0,0,1;
		return mat_i;
	}

	/** Return the se(3) matrix corresponding to a 6-vector twist V **/
	se3Mat VecTose3(Twist V){
		se3Mat mat;
                Vec3 omg(V(0), V(1), V(2));
                Vec3 p(V(3), V(4), V(5));
		mat << VecToso3(omg),p,
                       0,0,0,0; 
		return mat;
	}

	/** Returns Twist from se(3) matrix **/
	Twist se3ToVec(se3Mat mat){
		Twist V;
		V << mat(2, 1), mat(0, 2), mat(1, 0), mat(0, 3), mat(1, 3), mat(2, 3);
		return V;
	}

	/* Compute 6x6 adjoint matrix from T */
	AdjMat Adjoint(SE3Mat mat) {
		AdjMat adj_mat;
		so3Mat R = mat.block<3,3>(0,0);
		Vec3 p;
		p << mat(0,3), mat(1,3), mat(2,3);
		so3Mat vel_so3 = VecToso3(p);
		so3Mat mat_product = vel_so3 * R;
		adj_mat << R(0,0), R(0,1), R(0,2), 0, 0, 0,
			   R(1,0), R(1,1), R(1,2), 0, 0, 0,
			   R(2,0), R(2,1), R(2,2), 0, 0, 0,
			   mat_product(0,0), mat_product(0,1), mat_product(0,2), R(0,0), R(0,1), R(0,2),
			   mat_product(1,0), mat_product(1,1), mat_product(1,2), R(1,0), R(1,1), R(1,2),
			   mat_product(2,0), mat_product(2,1), mat_product(2,2), R(2,0), R(2,1), R(2,2);
		return adj_mat;
	}

	/** Returns a normalized screw axis representation **/
	ScrewAxis ScrewToAxis(Vec3 q, Vec3 s, double h) {
		ScrewAxis S;
                S.segment(0, 3) = s;
                S.segment(3, 3) = q.cross(s) + (h * s);
		return S;
	}

	/** Extracts the normalized screw axis S and the distance traveled along the screw theta from the 6-vector of exponential coordinates Stheta **/
	void AxisAng(Twist STheta, ScrewAxis &S, double &theta){
                Vec3 omg;
                omg << STheta(0),STheta(1),STheta(2);
                Vec3 vel;
                vel << STheta(3),STheta(4),STheta(5);
                theta = omg.norm();
                if (nearZero(theta))
                {
                    if(nearZero(vel.norm()))
                        theta = 1;
                    else
                        theta = vel.norm(); 
                }             
                S << STheta / theta;
		
	}

	/** Computes the homogeneous transformation matrix T \in SE(3) corresponding to matrix exponential of se3mat \in se(3) **/
	SE3Mat MatrixExp6(se3Mat in_mat){
		SE3Mat mat;
                so3Mat in_mat_cut = in_mat.block<3, 3>(0, 0);
                so3Mat mat_i;
                mat_i <<1,0,0,
                        0,1,0,
                        0,0,1; 
		Vec3 omg_theta = so3ToVec(in_mat_cut);

		if (nearZero(omg_theta.norm())) {

			in_mat_cut = mat_i;
			omg_theta << in_mat(0, 3), in_mat(1, 3), in_mat(2, 3);
			mat << in_mat_cut, omg_theta,
				0, 0, 0, 1;
			return mat;
		}

		else {

			AxisAngle angle = AxisAng3(omg_theta);

			so3Mat omg_mat = in_mat.block<3, 3>(0, 0) / angle.theta;
			so3Mat exp = mat_i * angle.theta + (1 - cos(angle.theta)) * omg_mat + ((angle.theta - sin(angle.theta)) * (omg_mat * omg_mat));

			Vec3 linear(in_mat(0, 3), in_mat(1, 3), in_mat(2, 3));
			Vec3 theta_v = (exp*linear) / angle.theta;

			mat << MatrixExp3(in_mat_cut), theta_v,
				0, 0, 0, 1;
			return mat;
               }
	}

	/** Computes the matrix logarithm se3mat \in se(3) of the homogeneous transformation matrix T \in SE(3) **/
	se3Mat MatrixLog6(SE3Mat T){
		se3Mat mat;
                SO3Mat R;
                Vec3 p;
                so3Mat mat_i;
                mat_i << 1,0,0,
                         0,1,0,
                         0,0,1;

		TransToRp(T,R,p);

		so3Mat omg_mat = MatrixLog3(R);
		so3Mat zeros_3d;
                zeros_3d << 0,0,0,
                            0,0,0,
                            0,0,0;
                if(omg_mat.isApprox(zeros_3d))
		{
		    mat << zeros_3d,p.normalized(),
		           0,0,0,0;
		}
                
                else
                {
                    double theta = acos((R.trace() - 1) / 2.0);
	            so3Mat log_exp_1 = mat_i - omg_mat / 2.0;
	            so3Mat log_exp_2 = (1.0 / theta - 1.0 / tan(theta / 2.0) / 2)*omg_mat*omg_mat / theta;
	            so3Mat log_exp = log_exp_1 + log_exp_2;
	            mat << omg_mat, log_exp*p,
		           0,0,0,0;
                }
		return mat;
	}

} /* IRlibrary */ 



#include "axxb_utils.h"

Eigen::Matrix3d skew(Eigen::Vector3d u)
{
  Eigen::Matrix3d u_hat = Eigen::MatrixXd::Zero(3,3);
  u_hat(0,1) = u(2);
  u_hat(1,0) = -u(2);
  u_hat(0,2) = -u(1);
  u_hat(2,0) = u(1);
  u_hat(1,2) = u(0);
  u_hat(2,1) = -u(0);

  return u_hat;
}




/** @brief converts a rotation matrix to ZXY Euler angles.
output rpy need to be in radians.
*/
Eigen::Vector3d rotation2EulerZXY(const Eigen::Matrix3d& R)
{
  #if 0
  double phi = asin(R(2, 1));
  // double theta = atan(-R(2, 0) / R(2, 2));
  // double psi = atan(-R(0, 1) / R(1, 1));
  double theta = atan2(-R(2, 0), R(2, 2));
  double psi = atan2(-R(0, 1), R(1, 1));

  return Eigen::Vector3d(phi, theta, psi);
  #endif


  #if 1
  const double pi = 3.14159265359;
  double thetaX, thetaY, thetaZ;
  if( R(2,1)< +1 ){

  	if( R(2,1) > -1 )
  	{
  		thetaX=asin(R(2,1)) ;
  		thetaZ=atan2(-R(0,1) , R(1,1) ) ;
  		thetaY=atan2(-R(2,0) , R(2,2) ) ;
  	}
  	else //  r21 =−1
  	{
  		// Not a  unique  s o l u t i o n :   thetaY−thetaZ = atan2 ( r02 , r00 )
  		thetaX =-pi /2;
  		thetaZ =-atan2( R(0,2) , R(0,0) ) ;
  		thetaY = 0;
  	}
  }
  else //  r21 = +1
  {
  	// Not a  unique  solution:   thetaY + thetaZ = atan2 ( r02 , r00 )
  	thetaX = +pi /2;
  	thetaZ = atan2(R(0,2),R(0,0)) ;
  	thetaY = 0;
  }

  double phi = thetaX;
  double theta = thetaY;
  double psi = thetaZ;
  return Eigen::Vector3d(phi, theta, psi);

  #endif


}

/** @brief converts a euler angle representation to rotation matrix
output rpy need to be in radians.
*/
Eigen::Matrix3d eulerZXY2Rotation( const Eigen::Vector3d& e  )
{

    double phi   = e(0);
    double theta = e(1);
    double psi   = e(2);


    Eigen::Quaterniond q = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()) *
                        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY());

    Eigen::Matrix3d Rzxy = q.toRotationMatrix();

    return Rzxy;
}


/** @brief given a roll-pitch-yaw returns a transformation matrix (4x4).
input rpy need to be in radians.
*/
Eigen::Matrix4d rpyt_to_transform( const Eigen::Vector3d& rpy, const Eigen::Vector3d& t )
{
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M.topLeftCorner<3,3>() = eulerZXY2Rotation( rpy );
    M.col(3) << t, 1.0;
    return M;
}


/** @brief given a transform returns a roll-pitch-yaw
output rpy need to be in radians.
*/
void transform_to_rpyt( const Eigen::Matrix4d& transform, Eigen::Vector3d& rpy, Eigen::Vector3d& t )
{
    Eigen::Matrix3d R = transform.topLeftCorner<3,3>();
    rpy = rotation2EulerZXY( R );
    t = transform.col(3).topRows(3);
}



Eigen::Vector3d deg2rads( Eigen::Vector3d& rpy_deg )
{
    const double _PI_ = 3.14159265359;
    return rpy_deg / 180. * _PI_;
}

Eigen::Vector3d rads2deg( Eigen::Vector3d& rpy )
{
    const double _PI_ = 3.14159265359;
    return rpy / _PI_ * 180.;
}



std::string prettyPrint( Eigen::Matrix4d& M, bool ang_in_deg, bool ang_in_rads )
{
    Eigen::Vector3d rpy_cap, t_cap;
    transform_to_rpyt( M, rpy_cap, t_cap );

    std::stringstream ss;

    if( ang_in_deg )
        ss << "rpy:\t" << rads2deg(rpy_cap).transpose() << ";\t";

    if( ang_in_rads )
        ss << "rpy:\t" << rpy_cap.transpose() << ";\t";

    ss << "t:\t" << t_cap.transpose() << ";";

    return ss.str();
}

std::string prettyPrint( Eigen::Matrix3d& R, bool ang_in_deg, bool ang_in_rads )
{
    Eigen::Vector3d rpy_cap;
    rpy_cap = rotation2EulerZXY( R );

    std::stringstream ss;

    if( ang_in_deg )
        ss << "rpy(deg):\t" << rads2deg(rpy_cap).transpose() << ";\t";

    if( ang_in_rads )
        ss << "rpy(rads):\t" << rpy_cap.transpose() << ";\t";


    return ss.str();
}

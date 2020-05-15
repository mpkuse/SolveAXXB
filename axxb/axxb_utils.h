#ifndef AXXB_UTILS_H
#define AXXB_UTILS_H

#include <iostream>
#include <string>

//#include<Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

Eigen::Matrix3d skew(Eigen::Vector3d u);


Eigen::Vector3d rotation2EulerZXY(const Eigen::Matrix3d& R);
Eigen::Matrix3d eulerZXY2Rotation( const Eigen::Vector3d& e  );
Eigen::Matrix4d rpyt_to_transform( const Eigen::Vector3d& rpy, const Eigen::Vector3d& t );
void transform_to_rpyt( const Eigen::Matrix4d& transform, Eigen::Vector3d& rpy, Eigen::Vector3d& t );


Eigen::Vector3d deg2rads( Eigen::Vector3d& rpy_deg );
Eigen::Vector3d rads2deg( Eigen::Vector3d& rpy );

std::string prettyPrint( Eigen::Matrix4d& M, bool ang_in_deg=true, bool ang_in_rads=false );
std::string prettyPrint( Eigen::Matrix3d& R, bool ang_in_deg=true, bool ang_in_rads=false );

#endif // AXXB_UTILS_H

#pragma once
#include <Eigen/Core>

inline double deg2rad(double &rad)
{
    return double(rad * M_PI / 180);
}

Eigen::Isometry3d createTransform(Eigen::Vector3d transl, Eigen::Vector3d rpy)
{
    // debugassert
    Eigen::Isometry3d M = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(deg2rad(rpy[0]), Eigen::Vector3d::UnitX() ) 
        * Eigen::AngleAxisd(deg2rad(rpy[1]), Eigen::Vector3d::UnitY() ) 
        * Eigen::AngleAxisd(deg2rad(rpy[2]), Eigen::Vector3d::UnitZ() );  
        
    return M.prerotate(rot).pretranslate(transl);
}
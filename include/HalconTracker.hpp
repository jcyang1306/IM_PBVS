#pragma once

#include <librealsense2/rs.hpp>
#include <cstdio>
#include <opencv2/opencv.hpp>   // Include OpenCV API   
#include <opencv2/calib3d.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/core/eigen.hpp> // MUST INCLUDE THIS AFTER <Eigen/Dense>

#include <boost/array.hpp>


class HalconTracker 
{

public:
    HalconTracker():
        rs_pipe_( NULL ),
        camera_matrix_( cv::Mat::eye(3, 3, CV_64F) ),
        distortion_coeffs_( cv::Mat::zeros(5, 1, CV_64F) ),
        pattern_size_( cv::Size(4, 4) ),
        tag_size_(0.008),
        pnp_solver_type_(6) // cv::SOLVEPNP_IPPE
    {
        createControlPoints();
    };
    ~HalconTracker() {};

    // cam_type, resolution, fps
    // get cam_matrix , dist_coeff
    bool connectCamera(int camera_type, cv::Size resolution, int refresh_rate);
    // getCameraInfo()

    // cv::Size, tag_size, pnp_sovler
    bool setTargetParams(cv::Size pattern_size, double tag_size, int pnp_solver_type);
    // getTargetConfig()

    // bool setDesiredPose(Eigen::Matrix4d cdMo);
    // bool setDesiredPose(Eigen::Vector3d tvec, Eigen::Vector3d rvec);
    // bool setGain();

    // get frame, detect, vis?
    // bool detect(cv::Mat &cMo);

    bool detect(cv::Mat &cMo, bool visualize);

    Eigen::Isometry3d detect(bool visualize, cv::Mat &img_out);

    // bool detect(Eigen::Matrix4d &cMo);
    // bool detect(Eigen::Vector3d &tvec, Eigen::Vector3d rvec); // xyz + nomalized_axis * angle
    // bool detect(Eigen::Vector3d &tvec, Eigen::Vector4d quat); // xyz + quaternion(xyzw)
    // Eigen::Matrix4d detect();
    
    void createControlPoints();

    // offline detect

protected:
    std::shared_ptr<rs2::pipeline> rs_pipe_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_; 
    cv::Size pattern_size_;
    double tag_size_;
    int pnp_solver_type_;
    std::vector<cv::Point3f> obj_pts_;
}; // class HalconTracker 


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
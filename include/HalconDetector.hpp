#pragma once

#include <librealsense2/rs.hpp>
#include <cstdio>
#include <opencv2/opencv.hpp>   // Include OpenCV API   
#include <opencv2/calib3d.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/core/eigen.hpp> // MUST INCLUDE THIS AFTER <Eigen/Dense>

#include <boost/array.hpp>


class HalconDetector 
{

public:
    HalconDetector():
        rs_pipe_( NULL ),
        camera_matrix_( cv::Mat::eye(3, 3, CV_64F) ),
        distortion_coeffs_( cv::Mat::zeros(5, 1, CV_64F) ),
        pattern_size_( cv::Size(7, 7) ),
        tag_size_(0.0025),
        pnp_solver_type_(6) // cv::SOLVEPNP_IPPE
    {
        createControlPoints();
    };
    ~HalconDetector() {};

    // cam_type, resolution, fps
    // get cam_matrix , dist_coeff
    bool connectCamera(int camera_type, cv::Size resolution, int refresh_rate);

    // cv::Size, tag_size, pnp_sovler
    bool setTargetParams(cv::Size pattern_size, double tag_size, int pnp_solver_type);

    bool capture(cv::Mat &img);

    Eigen::Isometry3d detect(cv::Mat &img_raw, bool &success);

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
}; // class HalconDetector 



#include "HalconTracker.hpp"


#define debug if (0) std::cout
#define debug_assert if (0) assert
bool undestorted = true;

bool HalconTracker::connectCamera(int camera_type, cv::Size resolution, int refresh_rate)
{
    // TODO 
    // Check camera_type here
    debug << "connecting to realsense camera \n";

    rs2::config pipe_config;
    pipe_config.enable_stream(RS2_STREAM_COLOR, resolution.width, resolution.height, RS2_FORMAT_BGR8, refresh_rate);

    rs_pipe_ = std::make_shared<rs2::pipeline>();
    // rs2::pipeline pipe_;
    rs2::pipeline_profile selection = rs_pipe_->start(pipe_config);

    for (int i = 0; i < 30; i++)
    {
        auto frames = rs_pipe_->wait_for_frames();   //Drop several frames for auto-exposure
    }

    // get intrinsic parameters
    rs2::frameset frames;
    frames = rs_pipe_->wait_for_frames();
    rs2::frame color = frames.get_color_frame();
    rs2::stream_profile cprofile = color.get_profile();
    rs2::video_stream_profile cvsprofile(cprofile);
    rs2_intrinsics color_intrin = cvsprofile.get_intrinsics();

    std::vector<double> cam_info_D_;
    boost::array<double, 9> cam_info_K_;

    // distortion coeefs
    cam_info_D_.clear();
    for (auto value : color_intrin.coeffs)
    {
        cam_info_D_.push_back(value);
    }

    // camera matrix
    cam_info_K_ = { color_intrin.fx,    0.,     color_intrin.ppx, 
                    0.,    color_intrin.fy,     color_intrin.ppy, 
                    0.,                 0.,                   1. };

    // Assign camera params
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
        camera_matrix_.at<double>(i,j) = cam_info_K_.at(i * 3 + j);
        }
    }

    for (int i = 0; i < 5; ++i)
    {
        distortion_coeffs_.at<double>(1, i) = undestorted ? 0.0 : cam_info_D_.at(i);
    }

    debug << "Connect to camera Success\n" 
          << "||Camera matrix||:\n" << camera_matrix_ << "\n"
          << "||Distortion coeff||:\n" << distortion_coeffs_ 
          << "\n";

    return true;
}

bool HalconTracker::setTargetParams(cv::Size pattern_size, double tag_size, int pnp_solver_type)
{
    // assert pnp type in range
    pattern_size_ = pattern_size;
    tag_size_ = tag_size;
    pnp_solver_type_ = pnp_solver_type;

    createControlPoints();
    return true;
}

void HalconTracker::createControlPoints()
{
    // creating 
    obj_pts_.clear();
    for (size_t idx = 0; idx < pattern_size_.width * pattern_size_.height; ++idx)
    {
        double dist_y = (idx % pattern_size_.width + 1) * tag_size_;
        double dist_x = floor(idx / pattern_size_.width + 1) * tag_size_;
        obj_pts_.push_back(cv::Point3f(dist_x, dist_y, 0));

        debug << "||idx|| " << idx << ", local coordinate: " << obj_pts_.back() << "\n";
    }
    return;
} 

bool HalconTracker::capture(cv::Mat &img)
{
    auto frames = rs_pipe_->wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();
    cv::Mat img_raw = cv::Mat(cv::Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    img = img_raw;

    return true;
}

Eigen::Isometry3d HalconTracker::detect(cv::Mat &img_raw, bool &success)
{
    // convert to gray if rgb input type
    cv::Mat img;
    if (img_raw.channels() == 3)
    {
        cv::cvtColor(img_raw, img, cv::COLOR_RGB2GRAY);
    }
    else 
    {
        img = img_raw;
    }

    std::vector<cv::Point2f> centers; //this will be filled by the detected centers
    bool patternfound = findCirclesGrid(img, pattern_size_, centers);
    if (!patternfound) 
    {
        std::cout << "Failed to detect pattern" << "\n";
        success = false;
        return Eigen::Isometry3d::Identity();
    }

    drawChessboardCorners(img_raw, pattern_size_, cv::Mat(centers), patternfound);
    // estimate pose
    cv::Vec3d tvec;
    cv::Vec3d rvec;
    bool useExtrinsicGuess = false;
    solvePnP(obj_pts_, centers, camera_matrix_, distortion_coeffs_, rvec, tvec, useExtrinsicGuess, cv::SOLVEPNP_IPPE);
    
    // Eigen format transformation
    Eigen::Vector3d eigenVec3; // working param 
    cv::cv2eigen(cv::normalize(rvec), eigenVec3);
    Eigen::Isometry3d cMo = Eigen::Isometry3d::Identity();
    cMo.prerotate( Eigen::AngleAxisd(cv::norm(rvec), eigenVec3) );
    cv::cv2eigen(tvec, eigenVec3);
    cMo.pretranslate(eigenVec3);
    debug << "||cMo|| \n" << cMo.matrix() << "\n";

    success = true;
    return cMo;
}

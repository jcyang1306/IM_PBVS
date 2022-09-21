#pragma once

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <thread>
#include <chrono>

#include "HalconTracker.hpp"
#include "pid.hpp"
#include "pbvs_helper.hpp"

#define ROBOTIP "192.168.1.50"

class PBVS
{

public:
    PBVS():
        cdMo_( Eigen::Isometry3d::Identity() ),
        eMc_( Eigen::Isometry3d::Identity() ),
        p_tracker( NULL ),
        rtde_c( NULL ),
        rtde_r( NULL ),
        oMo_tvec_( Eigen::Vector3d::Zero(3, 1) )
    {
        // Connect  camera?
        // UR RTDE init
        // Pid ctrl params
        // set gain?
        // cMcd eMc   

        // cMe_ = eMc.inverse();

        // Set pid controller params (dt, max, min, kp, kd, ki)
        double Dt = 1000 / 30;
        pid_tx = std::make_unique<PID> (Dt, 0.5, -0.5, 0.6, 0.05, 0.01);
        pid_ty = std::make_unique<PID> (Dt, 0.5, -0.5, 0.6, 0.05, 0.01);
        pid_tz = std::make_unique<PID> (Dt, 0.5, -0.5, 0.6, 0.05, 0.01);
        pid_rx = std::make_unique<PID> (Dt, 0.5, -0.5, 0.4, 0.05, 0.01);
        pid_ry = std::make_unique<PID> (Dt, 0.5, -0.5, 0.4, 0.05, 0.01);
        pid_rz = std::make_unique<PID> (Dt, 0.5, -0.5, 0.4, 0.05, 0.01);

        eMc_  = createTransform(Eigen::Vector3d(0, 0, 0), 
                                Eigen::Vector3d(180, 35, -90)); 

        p_tracker = std::make_shared<HalconTracker>();
    }
    ~PBVS() {};

    bool setGain();

    bool setTask(Eigen::Isometry3d  &cdMo);
    bool setTask(double offsetZ);

    bool connectRobot();

    bool connectCamera(int cam_type = 0, cv::Size resolution = cv::Size(1280, 720), int fps = 30);

    bool capture(cv::Mat &img);

    Eigen::Isometry3d detect(cv::Mat &img);

    std::vector<double> computeVc(Eigen::Isometry3d &cMo, double rot_thres);

    void stopAll();

    void speedL(std::vector<double> vc) { rtde_c->speedL(vc, 0.5); }

    void topdownReaching(double offsetZ);

    void sixDofReaching(int num_pts, double offsetZ);

    std::vector<double> servoReaching(int num_pts, double offsetZ);

    void logInfo()
    {
        std::cout << "\n\n\n-----LOGINFO-----" << "\n";
        std::cout  << "\n||cdMo||" << cdMo_.matrix() << "\n";
    }

    void setZoffset(double offsetZ) { oMo_tvec_[2] = offsetZ; }

protected:
    Eigen::Isometry3d cdMo_;
    Eigen::Vector3d oMo_tvec_;
    Eigen::Isometry3d eMc_;
    std::shared_ptr<HalconTracker> p_tracker;
    std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_c;
    std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_r;

    std::unique_ptr<PID> pid_tx;
    std::unique_ptr<PID> pid_ty;
    std::unique_ptr<PID> pid_tz;
    std::unique_ptr<PID> pid_rx;
    std::unique_ptr<PID> pid_ry;
    std::unique_ptr<PID> pid_rz;
}; // class HalconTracker 
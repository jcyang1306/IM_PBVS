#pragma once

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <thread>
#include <chrono>

#include "HalconTracker.hpp"
#include "pid.hpp"

#define ROBOTIP "192.168.1.50"

class PBVS
{

public:
    PBVS() {};
    ~PBVS() {};

    bool setGain();

    bool setTask(Eigen::Isometry3d  &cdMo);
    bool setTask();
    
    bool startTask();

    bool connectCamera(int cam_type, cv::Size resolution, int fps);



protected:
    Eigen::Isometry3d cdMo_;
    Eigen::Isometry3d eMc_;
    std::shared_ptr<HalconTracker> p_tracker;
}; // class HalconTracker 

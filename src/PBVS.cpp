#include "PBVS.hpp"


#define debug if (1) std::cout
#define debug_assert if (0) assert




bool PBVS::connectRobot()
{
    // Robot connection
    rtde_c = std::make_shared<ur_rtde::RTDEControlInterface> (ROBOTIP);
    std::vector<std::string> args({"timestamp", "actual_q", "actual_TCP_pose"});
    rtde_r = std::make_shared<ur_rtde::RTDEReceiveInterface> (ROBOTIP, 50, args) ; // need pass args)
    return true;
}

bool PBVS::connectCamera(int cam_type, cv::Size resolution, int fps)
{
    if (!p_tracker) return false;
    return p_tracker->connectCamera(cam_type, resolution, fps);
}

bool PBVS::capture(cv::Mat &img)
{
    if (!p_tracker) return false;
    return p_tracker->capture(img);
}

Eigen::Isometry3d PBVS::detect(cv::Mat &img)
{
    if (!p_tracker) return Eigen::Isometry3d::Identity();
    return p_tracker->detect(img);
}

std::vector<double> PBVS::computeVc(Eigen::Isometry3d &cMo, double rot_thres)
{
    // Eigen::Isometry3d cMo = p_tracker->detect(false, img);

    // pattern found, start servoing
    Eigen::Isometry3d cMcd = cMo * cdMo_.inverse();
    debug << "||cMcd|| \n" << cMcd.matrix() << "\n";

    std::vector<double> tcp_pose = rtde_r->getActualTCPPose();
    Eigen::Vector3d bMe_rvec = Eigen::Vector3d(tcp_pose[3], tcp_pose[4], tcp_pose[5]);
    Eigen::Isometry3d bMc = eMc_.inverse().prerotate( Eigen::AngleAxisd(bMe_rvec.norm(), bMe_rvec.normalized()) ); //rotation only
    debug << "||bMc|| \n" << bMc.matrix() << "\n";

    Eigen::Isometry3d err_mat = bMc * cMcd; //bMvc
    Eigen::AngleAxisd angvec_cMcd = Eigen::AngleAxisd(cMcd.rotation());
    Eigen::Vector3d err_rvec = angvec_cMcd.angle() * bMc.rotation() * angvec_cMcd.axis(); // rodrigues format
    Eigen::Vector3d err_tvec = bMc.rotation() * cMcd.translation();
    debug << "||errVec||: " << err_tvec.transpose() << ", " << err_rvec.transpose() << "\n";

    // Compute feedback control term
    // double rot_thres = 0.0001;
    Eigen::Vector3d rvec_f = Eigen::Vector3d::Zero(3, 1);
    rvec_f = err_tvec.isApprox(Eigen::Vector3d::Zero(3, 1), rot_thres) ? rvec_f : err_rvec; // keep curr rot if converge
    std::vector<double> vc{ pid_tx->calculate(err_tvec[0]), 
                            pid_ty->calculate(err_tvec[1]),
                            pid_tz->calculate(err_tvec[2]),
                            // rvec_f[0] * 0.4,
                            // rvec_f[1] * 0.4,
                            // rvec_f[2] * 0.4,
                            pid_rx->calculate(rvec_f[0]),
                            pid_ry->calculate(rvec_f[1]),
                            pid_rz->calculate(rvec_f[2]) 
                            };
    debug << "||vc||: ";
    for (auto val : vc) {debug << val << ", ";}
    debug << "\n";

    return vc;
}

bool PBVS::setTask()
{
    rtde_c->speedStop();
    cdMo_.setIdentity();
    Eigen::Vector3d t_term = Eigen::Vector3d::Zero(3,1);
    Eigen::Vector3d aVec = Eigen::Vector3d::Zero(3,1); // axis vec
    double angle = 0.0;
    for (int i = 0; i < 5; ++i) 
    {
        cv::Mat img;
        p_tracker->capture(img);
        Eigen::Isometry3d cMo_tmp = p_tracker->detect(img);
        t_term += cMo_tmp.translation();
        Eigen::AngleAxisd aVec_tmp = Eigen::AngleAxisd(cMo_tmp.rotation());
        aVec += aVec_tmp.axis();
        angle += aVec_tmp.angle();
    }
    aVec /= 5;
    angle /= 5;
    t_term /= 5;

    cdMo_.rotate( Eigen::AngleAxisd(angle, aVec.normalized()) );
    cdMo_.translation() = t_term; 
    debug << "||cdMo|| \n" << cdMo_.matrix() << "\n\n\n";

    return true;
}


void PBVS::reaching(double offsetZ)
{
    // assert(isconverged);

    // TODO: REACHING
    // While not converge, servoL
    rtde_c->speedStop();

    std::vector<double> tcp_tgt_pose = rtde_r->getActualTCPPose();
    tcp_tgt_pose[2] -= offsetZ;

    debug << "||target pose||: ";
    for (auto val : tcp_tgt_pose) {debug << val << ", ";}
    debug << "\n";

    rtde_c->moveL(tcp_tgt_pose, 0.02, 0.05);
    return;
}

void PBVS::stopAll()
{
    // assert(isconverged);
    rtde_c->stopL();
    rtde_c->stopScript();

    return;
}
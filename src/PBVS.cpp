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
    Eigen::Isometry3d cMcd = cMo * cdMo_.translate(oMo_tvec_).inverse();
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
                            pid_rx->calculate(rvec_f[0]),
                            pid_ry->calculate(rvec_f[1]),
                            pid_rz->calculate(rvec_f[2]) 
                            };
    debug << "||vc||: ";
    for (auto val : vc) {debug << val << ", ";}
    debug << "\n";

    return vc;
}

bool PBVS::setTask(double offsetZ)
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
    oMo_tvec_[2] = offsetZ;
    // cdMo_.translate(oMo_tvec_);
    debug << "||cdMo|| \n" << cdMo_.matrix() << "\n\n\n";

    return true;
}

void PBVS::topdownReaching(double offsetZ)
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

// tvec_projected = bMo_rot * zoffsetVec
// interpolation till tvec_projected = 0
void PBVS::sixDofReaching(int num_pts, double offsetZ)
{
    // assert (rot_converged)
    rtde_c->speedStop();

    // Compute avg cMo rotation, use angleAxis for linear interpolation
    Eigen::Vector3d axisVec = Eigen::Vector3d::Zero(3,1);
    double angle = 0.0;
    int num_samples = 5;
    for (int i = 0; i < num_samples; ++i) 
    {
        cv::Mat img;
        p_tracker->capture(img);
        Eigen::Isometry3d cMo_tmp = p_tracker->detect(img);
        Eigen::AngleAxisd aVec_tmp = Eigen::AngleAxisd(cMo_tmp.rotation());
        axisVec += aVec_tmp.axis();
        angle += aVec_tmp.angle();
    }
    axisVec /= num_samples;
    angle /= num_samples;

    // Project Zoffset to object coordinate
    std::vector<double> tcp_pose = rtde_r->getActualTCPPose();
    Eigen::Vector3d bMe_rvec = Eigen::Vector3d(tcp_pose[3], tcp_pose[4], tcp_pose[5]);
    Eigen::Isometry3d bMc = eMc_.inverse().prerotate( Eigen::AngleAxisd(bMe_rvec.norm(), bMe_rvec.normalized()) ); //rotation only
    Eigen::Isometry3d bMo = bMc.rotate(Eigen::AngleAxisd(angle, axisVec.normalized() ) );
    Eigen::Vector3d tvec_proj = bMo.rotation() * Eigen::Vector3d(0, 0, -offsetZ);
    debug << "||tvec_proj|| " << tvec_proj.transpose() << "\n";

    // Interpolate poses and MOVE THE ROBOT!!!
    for (int i = 0; i < num_pts; ++i)
    {
        tcp_pose[0] += tvec_proj[0] / num_pts;
        tcp_pose[1] += tvec_proj[1] / num_pts;
        tcp_pose[2] += tvec_proj[2] / num_pts;
        rtde_c->moveL(tcp_pose, 0.01, 0.02); // modify spd and acc here?
        debug << "idx " << i << "/" << num_pts << "\n";
    }

    return;
}

std::vector<double> PBVS::servoReaching(int num_pts)
{
    // assert(rot_converged)

    double offsetZ = oMo_tvec_[2];
    std::vector<double> waypoints;
    for (int i = 0; i < num_pts; ++i)
    {
        double pt = offsetZ * i  / num_pts ;
        waypoints.push_back(pt);
        debug << "idx " << i << "/" << num_pts << ": " << pt << "\n";
    }
    return waypoints;
}

void PBVS::stopAll()
{
    // assert(isconverged);
    rtde_c->stopL();
    rtde_c->stopScript();

    return;
}
#include "HalconDetector.hpp"

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <thread>
#include <chrono>

#include "pid.hpp"
#include "pbvs_helper.hpp"

#define debug if (1) std::cout
#define debug_assert if (0) assert

#define ROBOTIP "192.168.1.50"

void signal_handler(int signum) {
    printf("signal_handler: caught signal %d\n", signum);
    if (signum == SIGINT) {
        printf("SIGINT\n");
        exit(1);
    }
}

int main(int argc, char** argv) try
{
    // Camera
    HalconDetector detector;
    // cv resolution(1280, 720);
    detector.connectCamera(0, cv::Size(1280, 720), 30);

    Eigen::Isometry3d cdMo = createTransform(Eigen::Vector3d(0, 0, 0.2), //translation
                                             Eigen::Vector3d(180, 0, -90)); // rpy in degree
    Eigen::Isometry3d eMc  = createTransform(Eigen::Vector3d(0, 0, 0), 
                                             Eigen::Vector3d(180, 0, -90)); 
    
    debug << "||cdMo|| \n" << cdMo.matrix() << "\n";
    debug << "||eMc|| \n" << eMc.matrix() << "\n";

    // Robot connection
    ur_rtde::RTDEControlInterface rtde_control(ROBOTIP);
    ur_rtde::RTDEReceiveInterface rtde_receive(ROBOTIP, 50, {"timestamp", 
                                                             "actual_q", 
                                                             "actual_TCP_pose"}) ; // need pass args

    double Kp = 0.4;
    double Dt = 1000 / 30;
    
    // Set tasnlation pid controller params (dt, max, min, kp, kd, ki)
    PID pid_tx = PID(Dt, 0.5, -0.5, 0.6, 0.05, 0.01);
    PID pid_ty = PID(Dt, 0.5, -0.5, 0.6, 0.05, 0.01);
    PID pid_tz = PID(Dt, 0.5, -0.5, 0.6, 0.05, 0.01);


    if (signal(SIGINT, signal_handler) == SIG_ERR) {
        printf("Failed to caught signal\n");
    }

    cv::Mat img = cv::Mat(cv::Size(1280, 720), CV_8UC3);
    bool reaching = false;
    bool start_servoing = false;
    while(1)
    {
        auto start = std::chrono::high_resolution_clock::now();
        detector.capture(img);
        bool detected = true;
        Eigen::Isometry3d cMo = detector.detect(img, detected);
        cv::imshow("pbvs", img);
        char c = cv::waitKey(1);
        if (c == 'q')
        {
            std::cout << "-----EXIT-----" << "\n";
            rtde_control.speedStop();
            break;
        }

        else if (c == 'r')
        {
            std::cout << "-----REACHING-----" << "\n";
            rtde_control.speedStop();
            reaching = true;
            break;
        }

        else if (c == 't')
        {
            std::cout << "\n\n\n-----STORED CURRENT cdMo-----" << "\n";
            rtde_control.speedStop();
            cdMo.setIdentity();
            Eigen::Vector3d t_term = cMo.translation();

            for (int i = 0; i < 3; ++i) 
            {
                detector.capture(img);
                bool detected = true;
                Eigen::Isometry3d cMo_tmp = detector.detect(img, detected);
                t_term += cMo_tmp.translation();
            }
            t_term /= 4;

            cdMo = cMo;
            cdMo.translation() = t_term; 
            debug << "||cdMo|| \n" << cMo.matrix() << "\n\n\n";
            start_servoing = true;
            continue;
        }

        // stop if pattern lost
        if(cMo.isApprox( Eigen::Isometry3d::Identity() ))
        {
            rtde_control.speedL( std::vector<double> {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} );
            continue;
        }

        // pattern found, start servoing
        Eigen::Isometry3d cMcd = cMo * cdMo.inverse();
        debug << "||cMo|| \n" << cMo.matrix() << "\n";
        debug << "||cMcd|| \n" << cMcd.matrix() << "\n";

        std::vector<double> tcp_pose = rtde_receive.getActualTCPPose();
        Eigen::Vector3d bMe_rvec = Eigen::Vector3d(tcp_pose[3], tcp_pose[4], tcp_pose[5]);
        Eigen::Isometry3d bMc = eMc.inverse().prerotate( Eigen::AngleAxisd(bMe_rvec.norm(), bMe_rvec.normalized()) ); //rotation only
        debug << "||bMc|| \n" << bMc.matrix() << "\n";

        Eigen::Isometry3d err_mat = bMc * cMcd; //bMvc
        Eigen::AngleAxisd angvec_cMcd = Eigen::AngleAxisd(cMcd.rotation());
        Eigen::Vector3d err_rvec = angvec_cMcd.angle() * bMc.rotation() * angvec_cMcd.axis(); // rodrigues format
        Eigen::Vector3d err_tvec = bMc.rotation() * cMcd.translation();
        debug << "||errVec||: " << err_tvec.transpose() << ", " << err_rvec.transpose() << "\n";

        if (start_servoing) 
        {
            // Compute feedback control term
            double rot_thres = 0.0001;
            Eigen::Vector3d ctrl_rvec = Eigen::Vector3d::Zero(3, 1);
            ctrl_rvec = err_rvec.isApprox(Eigen::Vector3d::Zero(3, 1), rot_thres) ? 
                                            ctrl_rvec : err_rvec * Kp; // keep curr rot if converge
            std::vector<double> vc{pid_tx.calculate(err_tvec[0]), 
                                pid_ty.calculate(err_tvec[1]),
                                pid_tz.calculate(err_tvec[2]),
                                ctrl_rvec[0], ctrl_rvec[1], ctrl_rvec[2]};
            debug << "||vc||: ";
            for (auto val : vc) {debug << val << ", ";}
            debug << "\n";

            // Move the robot
            rtde_control.speedL(vc, 0.5); 
        }

        // Control step 
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end-start;
        int wait = Dt - duration.count(); 
        debug << "||step freuq||" << duration.count() << " ms\n";
        if (wait > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(wait));

    }

    if (reaching)
    {
        // TODO: REACHING
        // While not converge, servoL
        std::vector<double> tcp_tgt_pose = rtde_receive.getActualTCPPose();
        tcp_tgt_pose[2] -= 0.1;

        debug << "||target pose||: ";
        for (auto val : tcp_tgt_pose) {debug << val << ", ";}
        debug << "\n";

        rtde_control.moveL(tcp_tgt_pose, 0.1, 0.25);
    }

    rtde_control.stopScript();

    return 1;
}

catch(const rs2::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs2::error
    printf("rs2::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}
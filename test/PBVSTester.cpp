#include "PBVS.hpp"

#define debug if (1) std::cout
#define debug_assert if (0) assert
#define STATIC_CDMO

int main(int argc, char** argv)
{

    PBVS IMServo;
    if (!IMServo.connectCamera()) 
    {
        debug << "Connect camera failed\n";
        return 0;
    }
    if (!IMServo.connectRobot()) 
    {
        debug << "Connect camera failed\n";
        return 0;
    }


    bool stop = false;
    bool start_servoing = false;
    cv::Mat img = cv::Mat(cv::Size(1280, 720), CV_8UC3);
    double Dt = 1000 / 30;
    std::vector<double> servo_waypts;
    while (!stop) 
    {
        auto start = std::chrono::high_resolution_clock::now();

        IMServo.capture(img);
        bool detected = true;
        Eigen::Isometry3d cMo = IMServo.detect(img, detected);

        // Compute vc and apply to robot
        if( detected )
        {
            if (start_servoing && !servo_waypts.empty())
            {
                debug << "||curr zoffset||" << servo_waypts.back() << " ms\n";
                IMServo.setZoffset(servo_waypts.back());
                servo_waypts.pop_back();
            }

            std::vector<double> vc = IMServo.computeVc(cMo, 0.0001);
            if (start_servoing) IMServo.speedL(vc); 
        }

        cv::imshow("IMpbvs", img);
        char c = cv::waitKey(1);
        if (c == 'q')
        {
            std::cout << "-----EXIT-----" << "\n";
            IMServo.stopAll();
            stop = true;
            break;
        }

        else if (c == 't')
        {
            std::cout << "\n\n\n-----STORED CURRENT cdMo-----" << "\n";
            #if defined(STATIC_CDMO)
            {
                Eigen::Matrix4d cdMo_mat;
                cdMo_mat << 0.0124074,   0.999916,        0.0038176,   -0.00853325,
                            0.829526,    -0.0124248,      0.55833,     0.000244082,
                            0.55833,     -0.00376063,     -0.82961,    0.0352478,
                            0,           0,               0,           1;
                Eigen::Isometry3d cdMo_static(cdMo_mat);
                IMServo.setTask(cdMo_static, -0.05); // negative offsetZ

                continue;
            }
            #endif

            IMServo.setTask(0.05); // save curr pose as desired pose
            continue;
        }

        else if (c == 's')
        {
            std::cout << "\n\n\n-----START SERVOING-----" << "\n";
            start_servoing = true;
            continue;
        }

        else if (c == 'r')
        {
            std::cout << "-----REACHING-----" << "\n";
            IMServo.topdownReaching(0.05);
            servo_waypts = IMServo.servoReaching(200, 0.05);
            break;
        }

        // Control step 
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end-start;
        int wait = Dt - duration.count(); 
        debug << "||step freq||" << duration.count() << " ms\n";
        if (wait > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(wait));
        }

    }
    IMServo.logInfo();


    return 0;
}

#ifndef EYE_IN_HAND_HPP_
#define EYE_IN_HAND_HPP_

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "file_manip.hpp"
#include "common.hpp"

namespace sparkvis {
class EIHCalibrator {
public:
    EIHCalibrator();
    ~EIHCalibrator();
    EIHCalibrator(const EIHCalibrator& rhs);
    EIHCalibrator(EIHCalibrator&& rhs);
    EIHCalibrator& operator=(const EIHCalibrator& rhs);
    EIHCalibrator& operator=(EIHCalibrator&& rhs);
    
    class RobotPose {
    public:
        void updateTrans(double x, double y, double z);
        void updateQuat(double qw, double qx, double qy, double qz);
        void updateRotVec(double rx, double ry, double rz);
        void updateEulerAngle(double r, double p, double y);
        /**
         * @brief Get the Pose object
         * 
         * @param rot_rep rotation representation can be rvec(rotation vector), quat(quaternion)
         * @return std::vector<double> 
         */
        std::vector<double> getPose(std::string rot_rep = "rvec");
        std::vector<double> getPoseRPY(std::string rot_order = "xyz");
        Eigen::Isometry3f getHomoPose();
    private:
        double x{0}, y{0}, z{0};
        Eigen::Quaternionf quat = Eigen::Quaternionf::Identity();
        Eigen::AngleAxisf rot_vec = Eigen::AngleAxisf::Identity();
        Eigen::Matrix3f rot_mat = Eigen::Matrix3f::Identity();
        bool is_trans_set{false};
        bool is_rot_set{false};
    };

    
    bool loadImages(std::string dir);
    bool loadPoses(std::string dir);
    bool calibrateHandEye(cv::Mat& R_camera2gripper, cv::Mat& t_camera2gripper);
    bool validateResult();
private:
    std::vector<cv::Mat> m_input_imgs;
    std::vector<RobotPose> m_robot_poses;
};
}

#endif
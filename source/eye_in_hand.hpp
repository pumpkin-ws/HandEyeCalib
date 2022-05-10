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

    enum class Pattern {CHESSBOARD, SYMCIRCLE, ASYMCIRCLE};
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
    bool loadCamParams(std::string dir);
    bool calibrateHandEye(
        const std::string& img_dir, 
        const std::string& cam_params_dir, 
        const std::string& pose_dir, 
        const cv::Size& board_size, 
        const float& board_dim,
        const Pattern& p
    );
    bool validateResult();
    bool findBoardPose(
        const cv::Size& board_size, 
        const float& pattern_dim, 
        const std::vector<cv::Point2f>& tracked_centers,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        cv::Mat& rvec,
        cv::Mat& tvec
    );
    Eigen::Isometry3f getCalibResult();

    
    bool findPattern (
        const cv::Mat& input, 
        const cv::Size& grid_size, 
        const Pattern& p, 
        std::vector<cv::Point2f>& centers_output, 
        bool draw_result=false
    );

private:
    std::vector<cv::Mat> m_input_imgs;
    std::vector<RobotPose> m_robot_poses;
    cv::Mat m_R_camera2gripper;
    cv::Mat m_t_camera2gripper;
    cv::Mat m_intrinsics; // the camera intrinsics
    cv::Mat m_distortion; // the distortion coefficients
    cv::Size m_expect_size;
};
}

#endif
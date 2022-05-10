#ifndef CAMERA_INTRINSICS_HPP
#define CAMERA_INTRINSICS_HPP

#include <vector>
#include "common.hpp"
#include "opencv2/opencv.hpp"
#include "file_manip.hpp"

namespace sparkvis{
class CamCalib{
public:
    // include the big 5 here, constructor, destructor, copy constructor
    /**
     * @brief Construct a new Cam Calib object
     * 
     */
    CamCalib();
    /**
     * @brief Destroy the Cam Calib object
     * 
     */
    ~CamCalib();
    /**
     * @brief Construct a new Cam Calib object
     * lvalue copy assignment operator
     * 
     * @param rhs 
     */
    CamCalib(const CamCalib& rhs);
    /**
     * @brief Construct a new Cam Calib object
     * r value copy assignment operator
     * @param rhs 
     */
    CamCalib(CamCalib&& rhs);
    /**
     * @brief 
     * 
     * @param rhs 
     * @return CamCalib& 
     */
    CamCalib& operator=(const CamCalib& rhs);
    /**
     * @brief 
     * 
     * @param rhs 
     * @return CamCalib& 
     */
    CamCalib& operator=(CamCalib&& rhs);

    friend std::ostream& operator<< (std::ostream &os, CamCalib &cc) {
        if (cc.m_calib_performed == true) {
            os << "Calibration is not performed yet. Perform calibration to see result." << std::endl;
            return os;
        } else {
            os << "\033[1;32m------Camera Calibration Result------\033[0m\n";
            os << "Date of calibration: " << cc.m_time_of_calib << std::endl;
            os << "Camera intrinsic matrix: " << std::endl;
            os << cc.m_intrinsics << std::endl;
            os << "Distortion coefficients: " << std::endl;
            os << cc.m_distortion << std::endl;
            os << "\033[1;32m------Camera Calibration Result------\033[0m\n";
        }
    }

    /**
     * @brief Load images from the given directory
     * 
     * @param dir 
     * @return true 
     * @return false 
     */
    bool loadImages(std::string dir);
    enum class Pattern {CHESSBOARD, SYMCIRCLE, ASYMCIRCLE};
    bool findPattern (const cv::Mat& input, const cv::Size& grid_size, const Pattern& p, std::vector<cv::Point2f>& centers_output, bool draw_result=false);
    bool calibrateIntrinsics (const Pattern& p, const cv::Size& grid_size, double pattern_dim, cv::Mat& camera_mat, cv::Mat& distortion, std::vector<cv::Mat>& rvecs_out, std::vector<cv::Mat>& tvecs_out, bool save_result=false);
    
private:
    std::vector<cv::Mat> m_intrinsic_imgs;
    cv::Mat m_intrinsics, m_distortion;
    time_t m_rawtime;
    char* m_time_of_calib;
    bool m_calib_performed;
    /* 
    m_rms is the total sum of squared distances between the observed feature points imagePoints and
    the projected (using the current estimates for camera parameters and the poses) object points
    objectPoints 
    */
    double m_rms{0};
    std::string m_result_dir;
    std::vector<cv::Mat> m_rvecs, m_tvecs; // The rotation vector and the translation vector of the object pose in camera

};
}

#endif
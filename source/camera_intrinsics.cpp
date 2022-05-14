#include "camera_intrinsics.hpp"

namespace sparkvis{
    CamCalib::CamCalib() {
        this->m_calib_performed = false;
        this->m_result_dir = getenv("PWD");
    };

    CamCalib::~CamCalib() {

    };

    CamCalib::CamCalib(const CamCalib& rhs) {
        this->m_calib_performed = rhs.m_calib_performed;
        this->m_distortion = rhs.m_distortion;
        this->m_intrinsic_imgs = rhs.m_intrinsic_imgs;
        this->m_intrinsics = rhs.m_intrinsics;
        this->m_rawtime = rhs.m_rawtime;
        this->m_time_of_calib = rhs.m_time_of_calib;
    };

    CamCalib::CamCalib(CamCalib&& rhs) {
        std::swap(this->m_calib_performed, rhs.m_calib_performed);
        std::swap(this->m_distortion, rhs.m_distortion);
        std::swap(this->m_intrinsic_imgs, rhs.m_intrinsic_imgs);
        std::swap(this->m_intrinsics, rhs.m_intrinsics);
        std::swap(this->m_rawtime, rhs.m_rawtime);
        std::swap(this->m_time_of_calib, rhs.m_time_of_calib);
    };

    CamCalib& CamCalib::operator=(const CamCalib& rhs) {
        this->m_calib_performed = rhs.m_calib_performed;
        this->m_distortion = rhs.m_distortion;
        this->m_intrinsic_imgs = rhs.m_intrinsic_imgs;
        this->m_intrinsics = rhs.m_intrinsics;
        this->m_rawtime = rhs.m_rawtime;
        this->m_time_of_calib = rhs.m_time_of_calib;
        return *this;
    };
    
    CamCalib& CamCalib::operator=(CamCalib&& rhs) {
        // swapping should be performed here
        std::swap(this->m_calib_performed, rhs.m_calib_performed);
        std::swap(this->m_distortion, rhs.m_distortion);
        std::swap(this->m_intrinsic_imgs, rhs.m_intrinsic_imgs);
        std::swap(this->m_intrinsics, rhs.m_intrinsics);
        std::swap(this->m_rawtime, rhs.m_rawtime);
        std::swap(this->m_time_of_calib, rhs.m_time_of_calib);
        return *this;
    };

    bool CamCalib::loadImages(std::string dir) {
        m_intrinsic_imgs.clear();
        m_img_names.clear();
        std::string checkDirCommand{"[ -d " + dir + " ]"}; 
        int dir_exist = system(checkDirCommand.c_str());
        if (dir_exist == 0) {
            m_img_names = getAllFileName(dir, ".png");
            if (m_img_names.empty()) {
                m_img_names = getAllFileName(dir, ".jpg");
                if (m_img_names.empty()) {
                    LOG_RED("ONLY SUPPORT PNG AND JPG IMAGES. OR THE DIRECTORY DOES NOT CONTIAN IMAGES");
                    return false;
                } else {
                    LOG_GREEN("IMAGE NAMES LOADED SUCCESSFULLY");
                }
            }
            if (m_img_names.size() < 4) {
                LOG_RED("NEED AT LEAST 4 IMAGES IN THE DIRECTORY");
                return false;
            }
            for (int i = 0; i < m_img_names.size(); i++) {
                try{
                    cv::Mat img = cv::imread(dir + "/" + m_img_names[i]);
                    m_intrinsic_imgs.push_back(img.clone());
                } catch (cv::Exception& e) {
                    LOG_RED(e.what());
                    return false;
                }
            }
            LOG_GREEN("IMAGES LOADED SUCCESSFULLY");
            /* Check images for consistent size */
            cv::Size img_size = m_intrinsic_imgs[0].size();
            for (auto img : m_intrinsic_imgs) {
                if (img.size() != img_size) {
                    LOG_ERROR("IMAGES ARE OF DIFFERENT SIZE, CANNOT PERFORM CALIBRATION");
                    m_intrinsic_imgs.clear();
                    return false;
                }
            }

            return true;
        } else {
            LOG_ERROR("DIRECTORY DOES NOT EXIST");
            return false;
        }
    };

    bool CamCalib::findPattern(const cv::Mat& input, const cv::Size& grid_size, const Pattern& p, const std::string& img_name, std::vector<cv::Point2f>& centers_output, bool draw_result) {
        centers_output.clear();
        switch(p) {
            case Pattern::CHESSBOARD : {
                cv::Mat track_result;
                bool pattern_found = cv::findChessboardCorners(input, grid_size, centers_output);
                if (!pattern_found) {
                    LOG_RED("UNABLE TO FIND CHESSBOARD PATTERN FOR %s", img_name.c_str());
                    cv::imshow(img_name, input);
                    cv::waitKey(0);
                    cv::destroyWindow(img_name);
                    return false;
                } else {
                    // refine to subpixel accuracy
                    cv::Mat gray;
                    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
                    cv::cornerSubPix(
                        gray,
                        centers_output,
                        cv::Size(11, 11), 
                        cv::Size(-1, -1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01)
                    );
                    if (draw_result == true) {
                        cv::Mat display = input.clone();
                        cv::drawChessboardCorners(display, grid_size, centers_output, pattern_found);
                        cv::imshow("Found corners", display);
                        cv::waitKey(0);
                        cv::destroyWindow("Found corners");
                    }
                    return true;
                }

                break;
            }
            case Pattern::SYMCIRCLE : {
                cv::Mat track_result;
                bool pattern_found = cv::findCirclesGrid(input, grid_size, track_result);
                if (!pattern_found) {
                    LOG_RED("UNABLE TO FIND CIRCLE GRID PATTERN");
                    cv::imshow("Track Failed", input);
                    cv::waitKey(0);
                    cv::destroyWindow("Track Failed");
                    return false;
                } else {
                    cv::Mat display = input.clone();
                    for (int i = 0; i < track_result.rows; i++) {
                        centers_output.push_back(track_result.at<cv::Point2f>(0, i));
                    }
                    if (draw_result == true) {
                        cv::drawChessboardCorners(display, grid_size, centers_output, pattern_found);
                        cv::imshow("Found circles", display);
                        cv::waitKey(0);
                        cv::destroyWindow("Found circles");
                    }
                    return true;
                }

                break;
            }
            case Pattern::ASYMCIRCLE : {
                LOG_RED("ASYMMETRIC CIRCLES COMING SOON");
                return false;
                break;
            }
        }
    };

    bool CamCalib::calibrateIntrinsics(const Pattern& p, const cv::Size& grid_size, double pattern_dim, cv::Mat& camera_mat, cv::Mat& distortion, std::vector<cv::Mat>& rvecs_out, std::vector<cv::Mat>& tvecs_out, bool save_result) {
        /* Check if images are loaded */
        if (m_intrinsic_imgs.empty()) {
            LOG_ERROR("LOAD IMAGES BEFORE CALIBRATION");
            return false;
        }
        cv::Size image_size = m_intrinsic_imgs[0].size();
        switch(p) {
            case Pattern::CHESSBOARD : {
                std::vector<std::vector<cv::Point2f>> center_lists;
                std::vector<cv::Point2f> center_points;
                for (int i = 0; i < m_intrinsic_imgs.size(); i++) {
                    if (findPattern(m_intrinsic_imgs[i], grid_size, p, m_img_names[i], center_points)) {
                        center_lists.push_back(center_points);
                    } else {
                        LOG_RED("Find pattern failed.");
                        return false;
                    }
                }
                /* Generate object based on grid size and grid dimensions */
                std::vector<std::vector<cv::Point3f>> object_lists;
                std::vector<cv::Point3f> object_points;
                for (int i = 0; i < grid_size.height; i++) {
                    for (int j = 0; j < grid_size.width; j++) {
                        object_points.emplace_back(cv::Point3f(i*pattern_dim, j*pattern_dim, 0));
                    }
                }
                object_lists.resize(m_intrinsic_imgs.size(), object_points);
                /* Perform calibration and save results */
                try{
                    m_rms = cv::calibrateCamera(object_lists, center_lists, image_size, m_intrinsics, m_distortion, m_rvecs, m_tvecs);
                    time(&m_rawtime);
                    m_time_of_calib = asctime(localtime(&m_rawtime));
                    m_calib_performed = true;
                    /* SAVE THE PARAMETERS */
                    if (save_result == true) {
                        std::string fname_cam = this->m_result_dir + "/camera_parameters.yml";
                        cv::FileStorage fs(fname_cam, cv::FileStorage::WRITE);
                        fs << "CalibrationTime" << m_time_of_calib;
                        fs << "Intrinsic" << m_intrinsics;
                        fs << "Distortion" << m_distortion;
                        fs << "ImageSize" << image_size;
                        fs.release();

                        std::string fname_pose = this->m_result_dir + "/board_pose.yml";
                        cv::FileStorage fs_pose(fname_pose, cv::FileStorage::WRITE);
                        fs_pose << "CalibrationTime" << m_time_of_calib;
                        fs_pose << "Rotation_Vec" << m_rvecs;
                        fs_pose << "Translation_Vec" << m_tvecs;
                        fs.release();
                    }
                    return true;
                } catch (cv::Exception& e) {
                    LOG_RED(e.what());
                    return false;
                }
                break;
            }
            case Pattern::SYMCIRCLE : {
                std::vector<std::vector<cv::Point2f>> center_lists;
                std::vector<cv::Point2f> center_points;
                for (int i = 0; i < m_intrinsic_imgs.size(); i++) {
                    if (findPattern(m_intrinsic_imgs[i], grid_size, p, m_img_names[i], center_points)) {
                        center_lists.push_back(center_points);
                    } else {
                        LOG_RED("Find pattern failed.");
                        return false;
                    }
                }
                /* Generate object based on grid size and grid dimensions */
                std::vector<std::vector<cv::Point3f>> object_lists;
                std::vector<cv::Point3f> object_points;
                for (int i = 0; i < grid_size.height; i++) {
                    for (int j = 0; j < grid_size.width; j++) {
                        object_points.emplace_back(cv::Point3f(i*pattern_dim, j*pattern_dim, 0));
                    }
                }
                object_lists.resize(m_intrinsic_imgs.size(), object_points);
                /* Perform calibration and save results */
                try{
                    m_rms = cv::calibrateCamera(object_lists, center_lists, image_size, m_intrinsics, m_distortion, m_rvecs, m_tvecs);
                    time(&m_rawtime);
                    m_time_of_calib = asctime(localtime(&m_rawtime));
                    m_calib_performed = true;
                    /* SAVE THE PARAMETERS */
                    if (save_result == true) {
                        std::string fname_cam = this->m_result_dir + "/camera_parameters.yml";
                        cv::FileStorage fs(fname_cam, cv::FileStorage::WRITE);
                        fs << "CalibrationTime" << m_time_of_calib;
                        fs << "Intrinsic" << m_intrinsics;
                        fs << "Distortion" << m_distortion;
                        fs << "ImageSize" << image_size;
                        fs.release();

                        std::string fname_pose = this->m_result_dir + "/board_pose.yml";
                        cv::FileStorage fs_pose(fname_pose, cv::FileStorage::WRITE);
                        fs_pose << "CalibrationTime" << m_time_of_calib;
                        fs_pose << "Rotation_Vec" << m_rvecs;
                        fs_pose << "Translation_Vec" << m_tvecs;
                        fs.release();
                    }
                    return true;
                } catch (cv::Exception& e) {
                    LOG_RED(e.what());
                    return false;
                }
                break;
            }
            case Pattern::ASYMCIRCLE : {
                LOG_RED("ASYMETRIC CIRCLE COMING SOON");
                return false;
                break;
            }
            default: {
                return false;
            }
        }
        return false;
    };

    bool CamCalib::setResultDir(std::string dir) {
        if(!checkDirectoryExists(dir)) {
            LOG_BLUE("DIRECTOR DOES NOT EXIST, RESULT WILL BE SAVED IN %s", m_result_dir.c_str());
            return false;
        } else {
            m_result_dir = dir;
            LOG_GREEN("RESULT DIRECTORY CHANGED TO: %s", m_result_dir.c_str());
            return true;
        }
    }
}
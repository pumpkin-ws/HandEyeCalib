#include "eye_in_hand.hpp"

namespace sparkvis {

    EIHCalibrator::EIHCalibrator() {

    };
    EIHCalibrator::~EIHCalibrator() {

    }
    EIHCalibrator::EIHCalibrator(const EIHCalibrator& rhs) {

    };
    EIHCalibrator::EIHCalibrator(EIHCalibrator&& rhs) {

    };
    EIHCalibrator& EIHCalibrator::operator=(const EIHCalibrator& rhs) {

    };
    EIHCalibrator& EIHCalibrator::operator=(EIHCalibrator&& rhs) {

    };

    bool EIHCalibrator::loadImages(std::string dir) {
        m_input_imgs.clear();
        std::string checkDirCommand{"[ -d " + dir + " ]"}; 
        int dir_exist = system(checkDirCommand.c_str());
        if (dir_exist == 0) {
            std::vector<std::string> img_names = getAllFileName(dir, ".png");
            std::sort(img_names.begin(), img_names.end(), [](std::string name1, std::string name2)->bool {
                int idx1 = atoi(name1.substr(0, name1.find_first_of('_')).c_str());
                int idx2 = atoi(name2.substr(0, name2.find_first_of('_')).c_str());
                return idx1 < idx2;
            });
            if (img_names.empty()) {
                img_names = getAllFileName(dir, ".jpg");
                if (img_names.empty()) {
                    LOG_RED("ONLY SUPPORT PNG AND JPG IMAGES. OR THE DIRECTORY DOES NOT CONTIAN IMAGES");
                    return false;
                } else {
                    LOG_GREEN("IMAGE LOADED SUCCESSFULLY");
                }
            }
            LOG_GREEN("IMAGE LOADED SUCCESSFULLY");
            if (img_names.size() < 4) {
                LOG_RED("NEED AT LEAST 4 IMAGES IN THE DIRECTORY");
                return false;
            }
            for (int i = 0; i < img_names.size(); i++) {
                try{
                    cv::Mat img = cv::imread(img_names[i]);
                    m_input_imgs.push_back(img);
                } catch (cv::Exception& e) {
                    LOG_RED(e.what());
                    return false;
                }
            }
            /* Check images for consistent size */
            cv::Size img_size = m_input_imgs[0].size();
            for (auto img : m_input_imgs) {
                if (img.size() != img_size) {
                    LOG_ERROR("IMAGES ARE OF DIFFERENT SIZE, CANNOT PERFORM CALIBRATION");
                    m_input_imgs.clear();
                    return false;
                }
            }
            return true;
        } else {
            LOG_ERROR("DIRECTORY DOES NOT EXIST");
            return false;
        }
    };

    bool EIHCalibrator::loadPoses(std::string dir) {
        if (!checkDirectoryExists(dir)) {
            LOG_RED("DIRECTORY DOES NOT EXIST");
            return false;
        }; 
        cv::FileStorage fs(dir, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            LOG_RED("UNABLE TO OPEN YAML POSE FILE");
            return false;
        }
        int num_of_poses{0};
        std::string rot_type;
        try{
            fs["num_of_poses"] >> num_of_poses;
            fs["rot_type"] >> rot_type;
            for (int i = 0; i < num_of_poses; i++) {
                std::string pose_name = "pose" + std::to_string(i + 1);
                std::vector<double> pos_vals;
                fs[pose_name] >> pos_vals;
                RobotPose rp;
                rp.updateTrans(pos_vals[0], pos_vals[1], pos_vals[2]);
                if (rot_type == "rotation vector") {
                    rp.updateRotVec(pos_vals[3], pos_vals[4], pos_vals[5]);
                    m_robot_poses.push_back(rp);
                } else {
                    m_robot_poses.clear();
                    LOG_ERROR("ONLY SUPPORT ROTATION VECTOR FOR NOW. QUATERNION AND EULER SUPPORT COMING SOON.");
                    return false;
                }
            }
            fs.release();
            return true;
        } catch(cv::Exception& e) {
            m_robot_poses.clear();
            LOG_RED(e.what());
            fs.release();
            return false;
        } catch(std::exception& e) {
            m_robot_poses.clear();
            LOG_RED(e.what());
            fs.release();
            return false;
        }

    };

    bool EIHCalibrator::loadCamParams(std::string dir) {
        if(!checkDirectoryExists(dir)) {
            LOG_RED("DIRECTORY DOES NOT EXIST");
            return false;
        }
        cv::FileStorage fs(dir, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            LOG_RED("UNABLE TO OPEN YAML POSE FILE");
            return false;
        }
        try {
            fs["Intrinsic"] >> m_intrinsics;
            fs["Distortion"] >> m_distortion;
            fs["ImageSize"] >> m_expect_size;
            fs.release();
            return true;
        } catch (cv::Exception& e) {
            LOG_RED(e.what());
            return false;
        } catch (std::exception& e) {
            LOG_RED(e.what());
            return false;
        }
    }

    void EIHCalibrator::RobotPose::updateTrans(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->is_trans_set = true;
    };

    void EIHCalibrator::RobotPose::updateRotVec(double rx, double ry, double rz) {
        Eigen::Vector3f rot(rx, ry, rz);
        this->rot_vec.angle() = rot.norm();
        this->rot_vec.axis() = rot.normalized();
        this->quat = this->rot_vec;
        this->rot_mat = this->quat;
        this->is_rot_set = true;
    }

    void EIHCalibrator::RobotPose::updateQuat(double qw, double qx, double qy, double qz) {

    }

    void EIHCalibrator::RobotPose::updateEulerAngle(double r, double p, double y) {

    };

    std::vector<double> EIHCalibrator::RobotPose::getPose(std::string rot_rep) {
    }
    
    std::vector<double> EIHCalibrator::RobotPose::getPoseRPY(std::string rot_order) {

    }

    Eigen::Isometry3f EIHCalibrator::RobotPose::getHomoPose() {
        Eigen::Vector3f trans(this->x, this->y, this->z);
        Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
        pose.pretranslate(trans);
        pose.rotate(this->quat);
        return pose;
    }

    bool EIHCalibrator::calibrateHandEye(
        const std::string& img_dir, 
        const std::string& cam_params_dir, 
        const std::string& pose_dir,
        const cv::Size& board_size, 
        const float& board_dim,
        const Pattern& p
    ) {
        if (!loadImages(img_dir)) {
            LOG_RED("UNABLE TO LAOD IMAGES");
            return false;
        }
        if (!loadPoses(pose_dir)) {
            LOG_RED("UNABLE TO LOAD POSES");
            return false;
        }
        if (!loadCamParams(cam_params_dir)) {
            LOG_RED("UNABLE TO LOAD CAMERA PARAMETERS");
            return false;
        }
        if (m_input_imgs.size() == 0 || m_robot_poses.size() == 0) {
            LOG_RED("SOMETHING WRONG WHEN LOADING IMAGES OR ROBOT POSES");
            return false;
        }
        if (m_input_imgs.size() != m_robot_poses.size()) {
            LOG_RED("NUMBER OF IMAGES NEEDS TO AGREE WITH NUMBER OF POSES");
            return false;
        }
        for (int i = 0; i < m_input_imgs.size(); i++) {
            
        }
        
    };

    bool EIHCalibrator::findBoardPose(
        const cv::Size& board_size, 
        const float& pattern_dim, 
        const std::vector<cv::Point2f>& tracked_centers,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        cv::Mat& rvec,
        cv::Mat& tvec
    ) {
        /* Generate the object points based on the grid size and the grid dimensions */
        std::vector<cv::Point3f> object_points;
        for (int i = 0; i < board_size.height; i++) {
            for (int j = 0; j < board_size.width; j++) {
                object_points.emplace_back(i*pattern_dim, j*pattern_dim, 0);
            }
        }

        /* Calculate translation and rotation with solvePnP */
        cv::Mat rvector = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat tvector = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat tracked_centers_map = cv::Mat(1, tracked_centers.size(), CV_32FC2);
        for (int i = 0; i < tracked_centers.size(); i++) {
            tracked_centers_map.at<cv::Point2f>(0, i) = tracked_centers[i];
        }
        try {
            if (!cv::solvePnP(object_points, tracked_centers_map, camera_matrix, dist_coeffs, rvector, tvector)) {
                return false;
            };
            rvec = rvector.clone();
            tvec = tvector.clone();
            return true;
        } catch (cv::Exception& e) {
            LOG_RED(e.what());
            return false;
        } catch (std::exception& e) {
            LOG_RED(e.what());
            return false;
        }
    }

    bool EIHCalibrator::findPattern(
        const cv::Mat& input, 
        const cv::Size& grid_size, 
        const Pattern& p, 
        std::vector<cv::Point2f>& centers_output, 
        bool draw_result
    ) {
        centers_output.clear();
        switch(p) {
            case Pattern::CHESSBOARD : {
                cv::Mat track_result;
                bool pattern_found = cv::findChessboardCorners(input, grid_size, centers_output);
                if (!pattern_found) {
                    LOG_RED("UNABLE TO FIND CHESSBOARD PATTERN");
                    cv::imshow("Track Failed", input);
                    cv::waitKey(0);
                    cv::destroyWindow("Track Failed");
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
    }
}
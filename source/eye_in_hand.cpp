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
        return true;
    };

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
}
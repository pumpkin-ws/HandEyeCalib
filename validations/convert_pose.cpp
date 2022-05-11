#include "opencv2/opencv.hpp"
#include <fstream>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

struct RobotPose{
    double x{0}, y{0}, z{0}, rx{0}, ry{0}, rz{0};
};

cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d& theta, std::string rotation_order="xyz") {
    using std::sin; using std::cos;
    cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1,      0,               0,
                                             0,      cos(theta[0]),  -sin(theta[0]),
                                             0,      sin(theta[0]),   cos(theta[0]));

    cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(theta[1]),   0,   sin(theta[1]),
                                             0,               1,   0,
                                            -sin(theta[1]),   0,   cos(theta[1]));    

    cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(theta[2]),   -sin(theta[2]),   0,
                                             sin(theta[2]),    cos(theta[2]),   0,
                                             0,                0,               1);
    if (rotation_order == "zyx") {
        return R_x * R_y * R_z;
    } else if (rotation_order == "xyz") {
        return R_z * R_y * R_x;
    } else {
        fprintf(stderr, "wrong rotation order specified, should be xyz or zyx.\n");
        return cv::Mat();
    }                                                                                                                                                           
}

int main(int argc, char** argv) {

    auto str2pose = [](std::string line)mutable->RobotPose {
        RobotPose rp;
        double x = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());
        double y = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());
        double z = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());   
        double rx = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());   
        double ry = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());   
        double rz = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());   

        rp.x = x;
        rp.y = y;
        rp.z = z;
        rp.rx = rx;
        rp.ry = ry;
        rp.rz = rz;

        return rp;
    };

    std::string filename;
    std::getline(std::cin, filename);
    std::ifstream f(filename);
    std::string cur_line;

    std::vector<RobotPose> poses;
    int count = 0;
    while (getline(f, cur_line)) {
        if (cur_line[0] != '\0') {
            RobotPose rp = str2pose(cur_line);
            poses.push_back(rp);
            count++;
            std::cout << "Current pose " << count << ": " << std::endl;
            printf("x=%f, y=%f, z=%f, rx=%f, ry=%f, rz=%f.\n", rp.x, rp.y, rp.z, rp.rx, rp.ry, rp.rz);
        }
    }

    cv::FileStorage fs("robot_poses.yaml", cv::FileStorage::WRITE);
    
    fs << "num_of_poses" << 16;
    fs << "rot_type" << "rotation vector";
    fs << "distance_unit" << "mm";
    fs << "angle_unit" << "radian";

    for (int i = 0; i < poses.size(); i++) {
        cv::Vec3d rot(poses[i].rx, poses[i].ry, poses[i].rz);
        cv::Mat rot_mat = eulerAnglesToRotationMatrix(rot);
        Eigen::Matrix3f rot_eigen = Eigen::Matrix3f::Identity();
        rot_eigen << rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2), 
                     rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2), 
                     rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2);
        Eigen::AngleAxisf rot_vec(rot_eigen);
        printf("The rotation vector axis and angle are:\n");
        printf("Rotation axis:\n");
        std::cout << rot_vec.axis() << std::endl;
        printf("Rotation angle:\n");
        std::cout << rot_vec.angle() << std::endl;

        Eigen::Vector3f rvec = rot_vec.axis() * rot_vec.angle();
        std::vector<double> cur_pose{poses[i].x, poses[i].y, poses[i].z, rvec[0], rvec[1], rvec[2]};
        std::string posename = "pose" + std::to_string(i + 1);
        fs << posename << cur_pose;
    }
    fs.release();


    return EXIT_SUCCESS;
}
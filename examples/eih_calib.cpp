#include "eye_in_hand.hpp"

int main(int argc, char** argv) {
    sparkvis::EIHCalibrator eih;
    // eih.loadImages("./eih");
    // eih.loadCamParams("./camera_parameters.yml");
    // eih.loadPoses("./robot_poses.yaml");
    eih.setResultDir("./");
    eih.calibrateHandEye("./eih", "./camera_parameters.yml", "./robot_poses.yaml", cv::Size(9, 6), 18, "mm", sparkvis::EIHCalibrator::Pattern::CHESSBOARD);
    std::cout << eih << std::endl;
}
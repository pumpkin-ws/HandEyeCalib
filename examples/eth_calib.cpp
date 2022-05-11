#include "eye_to_hand.hpp"

int main(int argc, char** argv) {
    sparkvis::ETHCalibrator eth;
    // eih.loadImages("./eih");
    // eih.loadCamParams("./camera_parameters.yml");
    // eih.loadPoses("./robot_poses.yaml");
    eth.calibrateHandEye("./eth", "./camera_parameters.yml", "./robot_poses.yaml", cv::Size(9, 6), 18, "mm", sparkvis::ETHCalibrator::Pattern::CHESSBOARD);
    std::cout << eth << std::endl;
}
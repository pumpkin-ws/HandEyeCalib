#include "camera_intrinsics.hpp"

int main(int argc, char** argv) {
    sparkvis::CamCalib cc;
    std::cout << cc << std::endl;
    cc.loadImages("./eih/");
    cv::Mat intrinsics, distortion;
    std::vector<cv::Mat> rvecs, tvecs;
    cc.setResultDir("./"); // 这一步不是必要的，默认保存在运行文件的文件夹中
    cc.calibrateIntrinsics(sparkvis::CamCalib::Pattern::CHESSBOARD, cv::Size(9, 6), 18, intrinsics, distortion, rvecs, tvecs, true);
    std::cout << cc << std::endl;
}
#include "opencv2/opencv.hpp"

int main(int argc, char** argv) {
    printf("Opening file a.yml\n");
    cv::FileStorage fs("./a.yml", cv::FileStorage::READ);

    
    auto nodes = fs.getFirstTopLevelNode();
    std::cout << std::boolalpha << fs.isOpened() << std::endl;
    std::cout << nodes.isMap() << std::endl;
    std::cout << nodes.isNamed() << std::endl;
    std::cout << nodes.name() << std::endl;
    std::vector<double> vals;
    fs[nodes.name()] >> vals;
    std::cout << vals.size() << std::endl;
    for (auto elem : vals) {
        std::cout << elem << std::endl;
    }
    // for (auto
    std::string rot_type;
    fs["rot_type"] >> rot_type;
    std::cout << rot_type << std::endl;

}
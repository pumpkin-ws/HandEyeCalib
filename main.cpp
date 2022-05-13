#include <iostream>
// #include "sparkvis/spk.hpp"
#include "opencv2/opencv.hpp"

#define LOG_RED(...) do {\
    printf("\033[1;31mERROR: ")&&\
    printf(__VA_ARGS__)&&\
    printf("\033[0m\n");\
}while(0)

#define LOG_GREEN(...) do {\
    printf("\033[1;32mERROR: ")&&\
    printf(__VA_ARGS__)&&\
    printf("\033[0m\n");\
}while(0)

#define LOG_BLUE(...) do {\
    printf("\033[1;34mERROR: ")&&\
    printf(__VA_ARGS__)&&\
    printf("\033[0m\n");\
}while(0)

int main(int argc, char** argv) {
    
    std::cout << system("[ -d /usr ]") << std::endl;
    std::cout << system("echo hello world") << std::endl;

    LOG_RED("red message.");
    LOG_GREEN("green message.");
    LOG_BLUE("blue message.");
    std::cout << "\033[1;32m------Camera Calibration Result------\033[0m\n" << std::endl;
    std::cout << getenv("PWD") << std::endl;
    std::vector<int> vint {1, 2, 3, 4};
    cv::FileStorage fs("k.yml", cv::FileStorage::WRITE);
    fs << "vint" << vint;
    fs.release();

}

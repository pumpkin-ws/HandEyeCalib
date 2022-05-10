#include "file_manip.hpp"
bool checkTailValid(std::string tail) {
    bool valid = false;
    // check empty
    if(tail.empty()){
        std::cout << "empty extend name!" << std::endl;
    }
    else if(tail.at(0) != '.'){
        std::cout << "invalid extend name!" << std::endl;
    }
    else{
        valid = true;
    }
    return valid;
};

std::vector<std::string> getAllFileName(std::string path, std::string tail) {
    std::vector<std::string> fileNames;
    // TODO : needs to first check if path exists!
    // check valid extend name
    if(checkTailValid(tail) == false){
        return fileNames;
    }
    // check path empty 
    if(path.empty()){  
        char buff[PATH_MAX];
        getcwd(buff, PATH_MAX);
        path = buff;
        std::cout << "empty path! default pwd:" << path << " will be used to load file names" << std::endl;
    }
    
    // search file in directory
    for ( boost::filesystem::directory_iterator it(path); it != boost::filesystem::directory_iterator(); ++it ){
        if ( boost::filesystem::is_regular_file( it->status() ) && it->path().extension() == tail ){
            std::string fileName( it->path().filename().string() );
            fileNames.push_back(fileName);
        }
    }
    return fileNames;
};

bool checkDirectoryExists(std::string dir_name) {
    boost::filesystem::path p(dir_name);
    return boost::filesystem::exists(p);
}
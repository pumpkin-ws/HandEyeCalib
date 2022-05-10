#ifndef FILE_MANIP_HPP_
#define FILE_MANIP_HPP_

#include <string>
#include <iostream>
#include <vector>
#include "boost/filesystem.hpp"
#include "unistd.h"
#include <limits.h>

bool checkTailValid(std::string tail);
std::vector<std::string> getAllFileName(std::string path, std::string tail);
bool checkDirectoryExists(std::string dir_name);

#endif
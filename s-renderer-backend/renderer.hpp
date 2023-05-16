#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <string>

std::map<int, std::vector<uchar>> initialize_image_map(std::string obj_path, std::string obj_name, std::string texture_name, std::string shader);
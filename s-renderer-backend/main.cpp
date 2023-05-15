#include <cstdlib>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <restbed>
#include <sstream>
#include <iomanip>
#include <jsoncpp/json/json.h>
#include <map>
#include "renderer.hpp"

using namespace restbed;

std::map<std::string, std::map<int, std::vector<uchar>>> obj_image_map; 

void initialize_objs() {
    obj_image_map["Cow"] = initialize_image_map("../models/spot/", "spot_triangulated_good.obj", "hmap.jpg");
    // obj_image_map["Rock"] = initialize_image_map("../models/rock/", "rock.obj", "rock.png");
    // obj_image_map["Bunny"] = initialize_image_map("../models/rock/", "bunny.obj", "rock.png");

    return;
}

void get_image_handler(const std::shared_ptr<Session> session)
{
    const auto& request = session->get_request();
    const auto obj_param = request->get_path_parameter("object");
    const auto angle_param = request->get_path_parameter("angle");

    if (!angle_param.empty() && !obj_param.empty())
    {
        int angle = std::stoi(angle_param.c_str());

        std::vector<uchar> body = obj_image_map[obj_param][angle];
        session->close(OK, body, {{"Content-Type", "image/jpg"}});
        return;
    }
    session->close(BAD_REQUEST, "Invalid parameter.");
}

int main()
{
    initialize_objs();
    std::cout << "Image Map Setted" << std::endl;

    auto resource = std::make_shared<Resource>();
    resource->set_path("/image/{object: Bunny|Bear|Rock}/{angle:[0-9]{1,2}}");
    resource->set_method_handler("GET", get_image_handler);

    auto settings = std::make_shared<Settings>();
    settings->set_port(8080);
    settings->set_default_header("Connection", "keep-alive");
    settings->set_default_header( "Access-Control-Allow-Origin", "*");

    Service service;
    service.publish(resource);
    service.start(settings);

    return EXIT_SUCCESS;
}

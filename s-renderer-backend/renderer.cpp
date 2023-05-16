#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>


#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"
#define MAX_ANGLE 100

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * M_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    eye_fov = eye_fov * (M_PI / 180.0);
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f m_persp = Eigen::Matrix4f::Zero();
    m_persp(0, 0) = zNear;
    m_persp(1, 1) = zNear;
    m_persp(2, 2) = zNear + zFar;
    m_persp(2, 3) = - zNear * zFar;
    m_persp(3, 2) = 1;

    Eigen::Matrix4f m_ortho = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f m_ortho_translate = Eigen::Matrix4f::Identity();
    m_ortho_translate(2, 3) = - (zNear + zFar) / 2;

    Eigen::Matrix4f m_ortho_scale = Eigen::Matrix4f::Identity();
    m_ortho_scale(0, 0) = - 1 / (std::tan(eye_fov / 2) * std::abs(zNear) * aspect_ratio);
    m_ortho_scale(1, 1) = - 1 / (std::tan(eye_fov / 2) * std::abs(zNear));
    m_ortho_scale(2, 2) = 2 / (zNear - zFar); 

    projection = m_ortho_scale * m_ortho_translate * m_persp;

    return projection;


}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        return_color = payload.texture->getColor(payload.tex_coords.x(),payload.tex_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f ambient_light = ka.cwiseProduct(amb_light_intensity);

        Eigen::Vector3f intensity = light.intensity / (light.position - point).squaredNorm();
        
        Eigen::Vector3f diffuse_light = kd.cwiseProduct(intensity) * 
            std::max(0.0f, ((light.position - point).normalized()).dot(normal));

        Eigen::Vector3f bisector = (light.position - point).normalized() + (eye_pos - point).normalized();
        Eigen::Vector3f specular_light = ks.cwiseProduct(intensity) *
            std::pow(std::max(0.0f, (bisector.normalized()).dot(normal)), p);
        result_color += ambient_light + diffuse_light + specular_light;
    }
    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    
    for (auto& light : lights)
    {
        // For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f ambient_light = ka.cwiseProduct(amb_light_intensity);

        Eigen::Vector3f intensity = light.intensity / (light.position - point).squaredNorm();
        
        Eigen::Vector3f diffuse_light = kd.cwiseProduct(intensity) * 
            std::max(0.0f, ((light.position - point).normalized()).dot(normal));

        Eigen::Vector3f bisector = (light.position - point).normalized() + (eye_pos - point).normalized();
        Eigen::Vector3f specular_light = ks.cwiseProduct(intensity) *
            std::pow(std::max(0.0f, (bisector.normalized()).dot(normal)), p);
        result_color += ambient_light + diffuse_light + specular_light;
    }

    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    Eigen::Vector3f t = {x*y/std::sqrt(x*x+z*z) , std::sqrt(x*x+z*z) , z*y/std::sqrt(x*x+z*z)};
    Eigen::Vector3f b = normal.cross(t);
    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
            t.y(), b.y(), normal.y(),
            t.z(), b.z(), normal.z();


    int w = payload.texture->width;
    int h = payload.texture->height;
    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();

    payload.texture->getColor(u,v);

    auto huv = payload.texture->getColor(u,v).norm();
    
    float dU = kh * kn * (payload.texture->getColor(u+1.0f/w,v).norm()-huv);
    float dV = kh * kn * (payload.texture->getColor(u,v+1.0f/h).norm()-huv);

    Eigen::Vector3f ln(-dU,-dV,1);
    Eigen::Vector3f n = (TBN * ln).normalized();
    Eigen::Vector3f new_point = point +  n * huv * kn;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f ambient_light = ka.cwiseProduct(amb_light_intensity);

        Eigen::Vector3f intensity = light.intensity / (light.position - new_point).squaredNorm();
        
        Eigen::Vector3f diffuse_light = kd.cwiseProduct(intensity) * 
            std::max(0.0f, ((light.position - new_point).normalized()).dot(normal));

        Eigen::Vector3f bisector = (light.position - new_point).normalized() + (eye_pos - new_point).normalized();
        Eigen::Vector3f specular_light = ks.cwiseProduct(intensity) *
            std::pow(std::max(0.0f, (bisector.normalized()).dot(normal)), p);
        result_color += ambient_light + diffuse_light + specular_light;
    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    Eigen::Vector3f t = {x*y/std::sqrt(x*x+z*z) , std::sqrt(x*x+z*z) , z*y/std::sqrt(x*x+z*z)};
    Eigen::Vector3f b = normal.cross(t);
    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
            t.y(), b.y(), normal.y(),
            t.z(), b.z(), normal.z();


    int w = payload.texture->width;
    int h = payload.texture->height;
    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();

    payload.texture->getColor(u,v);

    auto huv = payload.texture->getColor(u,v).norm();
    
    float dU = kh * kn * (payload.texture->getColor(u+1.0f/w,v).norm()-huv);
    float dV = kh * kn * (payload.texture->getColor(u,v+1.0f/h).norm()-huv);

    Vector3f ln(-dU,-dV,1);
    Vector3f n = (TBN * ln).normalized();

    Eigen::Vector3f result_color = n;

    return result_color * 255.f;
}

std::map<int, std::vector<uchar>> initialize_image_map(std::string obj_path, std::string obj_name, std::string texture_name, std::string shader)
{
    std::vector<Triangle*> TriangleList;

    objl::Loader Loader;

    // Load .obj File
    bool loadout = Loader.LoadFile(obj_path + obj_name);
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    r.set_texture(Texture(obj_path + texture_name));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (shader == "displacement") {
        active_shader = displacement_fragment_shader;
    }
    
    if (shader == "texture") {
        active_shader = texture_fragment_shader;
    }
    if (shader == "bump") {
        active_shader = bump_fragment_shader;
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    std::map<int, std::vector<uchar>> image_map; 

    for (int input_angle = 0 ; input_angle < MAX_ANGLE; input_angle++) {
        float angle = (float)input_angle * 360.f / 100.f;

        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        for (auto & pixel: r.frame_buffer()) {
            if (pixel.isApprox(Eigen::Vector3f(0, 0, 0))) {
                pixel << 253, 199, 250;
            }
        }
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);


        std::vector<uchar> byteData;  

        cv::imencode(".png", image, byteData);

        image_map[input_angle] = byteData;
    }
    return std::move(image_map);
}

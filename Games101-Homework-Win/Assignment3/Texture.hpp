//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);
        auto u_img = u * width;
        auto v_img = (1. - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {   
        if (u > 1.)u = 1.;
        if (v > 1.) v = 1.;
        int ulow =std::floor(u*width);
        int vlow =std::floor((1. - v)*height);
        int uhigh =ulow+1;
        int vhigh =vlow+1;
        if(uhigh>width-1) uhigh = width-1; 
        if(vhigh>height-1) vhigh = height-1; 
        if(ulow<0) uhigh = 0; 
        if(vlow<0) uhigh = 0; 
        float w1 = u*width-ulow;
        float w2 = (1. - v)*height-vlow;
        auto color1 = image_data.at<cv::Vec3b>(vlow, ulow);
        auto color2 = image_data.at<cv::Vec3b>(vlow, uhigh);
        auto color3 = image_data.at<cv::Vec3b>(vhigh, ulow);
        auto color4 = image_data.at<cv::Vec3b>(vhigh, uhigh);
        auto color = color1*(1.0-w1)*(1.0-w2) + color2*w1*(1.0-w2) + color3*(1.0-w1)*w2 + color4*w1*w2;

        return Eigen::Vector3f(color[0], color[1], color[2]); 
    }

    std::pair<Eigen::Vector3f, Eigen::Vector3f> getGradiant(float u, float v)
    {
        if (u > 1.) u = 1.;
        if (v > 1.) v = 1.;
        if (u < 0.) u = 0.;
        if (v < 0.) v = 0.;
        int ulow =std::floor(u*width);
        int vlow =std::floor((1.0-v)*height);
        int uhigh =ulow+1;
        int vhigh =vlow+1;
        if(uhigh>width-1) uhigh = width-1; 
        if(vhigh>height-1) vhigh = height-1;
        if(vhigh<0) vhigh = 0;
        if(ulow<0) ulow = 0; 
        if(vlow<0) vlow = 0;
        auto color1 = image_data.at<cv::Vec3b>(vlow, ulow);
        auto color2 = image_data.at<cv::Vec3b>(vlow, uhigh);
        auto color3 = image_data.at<cv::Vec3b>(vhigh, ulow);
        return std::make_pair(Eigen::Vector3f(color2[0]-color1[0], color2[1]-color1[1], color2[2]-color1[2]), Eigen::Vector3f(color3[0]-color1[0], color3[1]-color1[1], color3[2]-color1[2]));
    }

};
#endif //RASTERIZER_TEXTURE_H

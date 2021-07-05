//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        int u_img = u * width;
        int v_img = (1 - v) * height;
        v_img = std::min(height - 1, v_img); // if v = 0, cause error
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    static cv::Vec3b lerp(float p, cv::Vec3b p0, cv::Vec3b p1)
    {
        return p0 + p * (p1 - p0);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u * width;
        float v_img = (1 - v) * height;

        int x = std::round(u_img);
        int y = std::round(v_img);

        x = std::max(0, x);
        y = std::max(0, y);

        auto useXLerp = x > 0 && x < width;
        auto useYLerp = y > 0 && y < height;

        if(useXLerp && useYLerp)
        {
            auto u00 = Eigen::Vector2f(x - 0.5f, y - 0.5f);
            auto u10 = Eigen::Vector2f(x + 0.5f, y - 0.5f);
            auto u11 = Eigen::Vector2f(x + 0.5f, y + 0.5f);
            auto u01 = Eigen::Vector2f(x - 0.5f, y + 0.5f);

            auto dx = u_img - (x - 0.5f);
            auto dy = v_img - (y - 0.5f);

            auto color00 = image_data.at<cv::Vec3b>(u00.y(), u00.x());
            auto color10 = image_data.at<cv::Vec3b>(u10.y(), u10.x());
            auto color11 = image_data.at<cv::Vec3b>(u11.y(), u11.x());
            auto color01 = image_data.at<cv::Vec3b>(u01.y(), u01.x());

            auto color0 = lerp(dx, color00, color10);
            auto color1 = lerp(dx, color01, color11);

            auto color = lerp(dy, color0, color1);
            return Eigen::Vector3f(color[0], color[1], color[2]);
        }
        else if(useYLerp)
        {
            auto u0 = Eigen::Vector2f(x, y - 0.5f);
            auto u1 = Eigen::Vector2f(x, y + 0.5f);

            auto dy = v_img - (y - 0.5f);

            auto color0 = image_data.at<cv::Vec3b>(u0.y(), u0.x());
            auto color1 = image_data.at<cv::Vec3b>(u1.y(), u1.x());

            auto color = lerp(dy, color0, color1);
            return Eigen::Vector3f(color[0], color[1], color[2]);
        }
        else if(useXLerp)
        {
            auto u0 = Eigen::Vector2f(x - 0.5f, y);
            auto u1 = Eigen::Vector2f(x + 0.5f, y);

            auto dx = u_img - (x - 0.5f);

            auto color0 = image_data.at<cv::Vec3b>(u0.y(), u0.x());
            auto color1 = image_data.at<cv::Vec3b>(u1.y(), u1.x());

            auto color = lerp(dx, color0, color1);
            return Eigen::Vector3f(color[0], color[1], color[2]);
        }
        else
        {
            auto color = image_data.at<cv::Vec3b>(v_img, u_img);
            return Eigen::Vector3f(color[0], color[1], color[2]);
        }
    }
};
#endif //RASTERIZER_TEXTURE_H

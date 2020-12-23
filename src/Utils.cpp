//
// Created by Simon on 09/11/2020.
//

#include <cmath>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "../headers/Utils.hpp"

namespace arfs
{

    double Utils::angleBetween(const cv::Point& p1, const cv::Point& p2, AngleType type)
    {
        double angle = acos(p1.dot(p2) / (cv::norm(p1) * cv::norm(p2)));
        if(type == AngleType::DEG)
            angle = angle * 180 / CV_PI;

        return angle;
    }

    double Utils::angleBetween(const cv::Vec3d& v1, const cv::Vec3d& v2, AngleType type)
    {
        double angle = acos(v1.dot(v2) / (cv::norm(v1) * cv::norm(v2)));
        if(type == AngleType::DEG)
            angle = angle * 180 / CV_PI;

        return angle;
    }

    double Utils::norm(const cv::Point& p1, const cv::Point& p2)
    {
        return cv::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    void Utils::saveImage(const cv::Mat& img, const std::string& filename)
    {
        cv::imwrite(filename, img);
    }

    void Utils::showImage(const cv::Mat& img, const std::string& winname)
    {
        cv::imshow(winname, img);
    }

    cv::Mat Utils::loadImage(const std::string& filename)
    {
        return cv::imread(filename);
    }

    std::vector<std::string> Utils::split(std::string s, const char& delimiter)
    {
        std::vector<std::string> split_strings{};
        size_t pos;
        std::string token{};
        while((pos = s.find(delimiter)) != std::string::npos)
        {
            token = s.substr(0, pos);
            split_strings.emplace_back(token);
            s.erase(0, pos + 1);
        }
        split_strings.emplace_back(s);

        return split_strings;
    }
}
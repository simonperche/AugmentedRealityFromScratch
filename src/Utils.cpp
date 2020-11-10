//
// Created by Simon on 09/11/2020.
//

#include <cmath>
#include <opencv2/imgcodecs.hpp>
#include "../headers/Utils.hpp"

namespace arfs
{

    double Utils::angleBetween(const cv::Point& p1, const cv::Point& p2)
    {
        double angle = atan2(p1.y - p2.y, p1.x - p2.x);

        if(angle < 0)
            angle += 2 * Utils::PI;

        return angle;
    }

    void Utils::saveImage(const cv::Mat& img, const std::string& filename)
    {
        cv::imwrite(filename, img);
    }
}
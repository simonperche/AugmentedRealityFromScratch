//
// Created by Simon on 09/11/2020.
//

#ifndef AR_UTILS_HPP
#define AR_UTILS_HPP

#include <opencv2/core/types.hpp>

namespace arfs
{
    enum AngleType
    {
        DEG,
        RAD
    };

    class Utils
    {
    public:
        static double angleBetween(const cv::Point& v1, const cv::Point& v2, AngleType type=AngleType::RAD);

        // cv::norm is not implemented with two points
        static double norm(const cv::Point& p1, const cv::Point& p2);

        static void saveImage(const cv::Mat& img, const std::string& filename);

        static void showImage(const cv::Mat& img, const std::string& winname);

        static constexpr double PI = 3.141592653589793238463;
    };
}

#endif //AR_UTILS_HPP

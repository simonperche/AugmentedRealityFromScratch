//
// Created by Simon on 09/11/2020.
//

#ifndef AR_UTILS_HPP
#define AR_UTILS_HPP

#include <opencv2/core/types.hpp>

namespace arfs
{
    class Utils
    {
    public:
        static double angleBetween(const cv::Point &v1, const cv::Point &v2);
        static constexpr double PI = 3.141592653589793238463;
    };
}

#endif //AR_UTILS_HPP

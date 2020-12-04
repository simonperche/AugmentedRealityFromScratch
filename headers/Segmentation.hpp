//
// Created by Simon on 10/11/2020.
//

#ifndef AR_SEGMENTATION_HPP
#define AR_SEGMENTATION_HPP

#include <opencv2/core/mat.hpp>

namespace arfs
{
    class Segmentation
    {
    public:
        static cv::Mat segmentation(const cv::Mat& img, const std::vector<std::pair<cv::Point2d, double>>& depthPoints);
        static cv::Mat extractTag(const cv::Mat& frame);
    private:
        static cv::Mat threshold(const cv::Mat& img);
    };
}

#endif //AR_SEGMENTATION_HPP

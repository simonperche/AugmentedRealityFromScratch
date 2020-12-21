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
        static std::vector<std::vector<cv::Point>> extractTagCandidates(const cv::Mat& frame);
        static std::vector<cv::Point> recognizeTag(const cv::Mat& frame, std::vector<std::vector<cv::Point>> candidates, std::array<int, 64> code);
        static std::array<int, 64> getARTagCode(const cv::Mat& tag_img);
        static cv::Mat getProjectionMatrix(const cv::Mat& homography, const cv::Matx33d& intrinsicMatrix);
        static std::vector<cv::Point2d> projectPoint(const std::vector<cv::Point3d>& points, const cv::Mat& projectionMatrix);
    private:
        static cv::Mat threshold(const cv::Mat& img);
        static void showAxis(const cv::Mat& homography, const std::vector<cv::Point>& tag, cv::Mat& frame);
    };
}

#endif //AR_SEGMENTATION_HPP

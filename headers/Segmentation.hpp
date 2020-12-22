//
// Created by Simon on 10/11/2020.
//

#ifndef AR_SEGMENTATION_HPP
#define AR_SEGMENTATION_HPP

#include <opencv2/core/mat.hpp>
#include "../headers/OBJLoader.h"

namespace arfs
{
    class Segmentation
    {
    public:
        static std::vector<std::vector<cv::Point>> extractTagCandidates(const cv::Mat& frame);
        static std::vector<cv::Point> recognizeTag(const cv::Mat& frame, const std::vector<std::vector<cv::Point>>& candidates, std::array<int, 64> code);
        static std::array<int, 64> getARTagCode(const cv::Mat& tag_img);
        static cv::Mat getProjectionMatrix(const cv::Mat& homography, const cv::Matx33d& intrinsicMatrix);
        static std::vector<cv::Point2i> projectPoint(const std::vector<cv::Point3d>& points, const cv::Mat& projectionMatrix);
        static void augmentObject(const OBJLoader& obj, const cv::Mat& frame, const std::vector<cv::Point>& tag);
    private:
        static cv::Mat threshold(const cv::Mat& img);
        static void showAxis(const cv::Mat& homography, const std::vector<cv::Point>& tag, cv::Mat& frame);

        //Generate points inside polygon to increase accuracy
        static cv::Mat computeHomography(const std::vector<cv::Point>& srcPoints);

        static const int m_tagSize = 300;
    };
}

#endif //AR_SEGMENTATION_HPP

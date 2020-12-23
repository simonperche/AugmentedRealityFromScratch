//
// Created by Simon on 22/12/2020.
//

#ifndef AR_TAGDETECTION_HPP
#define AR_TAGDETECTION_HPP

#include <vector>
#include <opencv2/core.hpp>

#include "ARTag.hpp"
#include "Tracking.hpp"

namespace arfs
{
    class TagDetection
    {
    public:
        explicit TagDetection(const arfs::ARTag& tag, bool verbose = false) : m_tagToDetect(tag), m_verbose(verbose)
        {};

        std::vector<cv::Point> update(const cv::Mat& frame);

    private:
        arfs::ARTag m_tagToDetect;
        std::vector<cv::Point> m_tagCorners{};
        arfs::Tracking m_tracking{};
        bool m_verbose;

        void fullDetection(const cv::Mat& frame);

        static std::vector<std::vector<cv::Point>> extractTagCandidates(const cv::Mat& frame);

        bool recognizeTag(const cv::Mat& frame, std::vector<cv::Point>& candidate);
    };
}

#endif //AR_TAGDETECTION_HPP

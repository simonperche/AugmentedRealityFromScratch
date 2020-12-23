//
// Created by Simon on 06/11/2020.
//

#ifndef AR_TRACKING_HPP
#define AR_TRACKING_HPP

#include <opencv2/core/types.hpp>
#include <opencv2/tracking.hpp>
#include <vector>

namespace arfs
{
    class Tracking
    {
    public:
        void addPoints(const std::vector<cv::Point>& points, const cv::Mat& initFrame);

        std::vector<cv::Point> update(const cv::Mat& frame);

        void showTrackedPoint(const cv::Mat& frame);

        void clear()
        { m_multiTracker.clear(); }

    private:
        struct Tracker
        {
            cv::Ptr<cv::Tracker> tracker;
            cv::Rect2d ROI;

            cv::Point2d point() const
            {
                return (ROI.br() + ROI.tl()) * 0.5;
            };
        };

        static constexpr unsigned int m_roiSize{20};

        // I didn't use cv::MultiTracker because I needed to know if the tracking is lost
        std::vector<Tracker> m_multiTracker{};
    };
}

#endif //AR_TRACKING_HPP

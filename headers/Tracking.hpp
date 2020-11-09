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
        explicit Tracking(const cv::Mat& frame);
        void update(const cv::Mat& frame);
        void showTrackedPoint(const cv::Mat& frame);
    private:
        struct TrackedPoint
        {
            cv::Point2d lastPosition{};
            double lastAngle{0};

            //Distances in pixels
            double totalDistance{0};
            double lastDistance{0};

            bool isTracked{true};
        };

        struct Tracker
        {
            cv::Ptr<cv::Tracker> tracker;
            cv::Rect2d ROI;

            cv::Point2d findCenter()
            {
                return (ROI.br() + ROI.tl()) * 0.5;
            };
        };

        static constexpr unsigned int m_roiSize = 30;
        static constexpr unsigned int m_nbTrackersMax = 1000;
        std::vector<cv::Point2d> m_corners{};
        // I didn't use cv::MultiTracker because I needed to know if the tracking is lost
        std::vector<Tracker> m_multiTracker{};
        std::vector<TrackedPoint> m_trackedPoints{};
    };
}

#endif //AR_TRACKING_HPP

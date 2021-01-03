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
    /**
     * @brief Tracking wrapper of opencv tracking module
     */
    class Tracking
    {
    public:
        /**
         * @brief Add tracking points. Need an frame to initialize trackers.
         * @param points
         * @param initFrame
         */
        void addPoints(const std::vector<cv::Point>& points, const cv::Mat& initFrame);

        /**
         * @brief Update the tracking and return new points
         * @param frame frame used to update the tracked points
         * @return new positions of tracked points
         */
        std::vector<cv::Point> update(const cv::Mat& frame);

        /**
         * @brief Draw tracked points on frame.
         * @param frame
         */
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

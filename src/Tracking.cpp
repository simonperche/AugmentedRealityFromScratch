//
// Created by Simon on 06/11/2020.
//

#include <opencv2/highgui.hpp>
#include <cmath>
#include "../headers/Tracking.hpp"
#include "../headers/Utils.hpp"

namespace arfs
{
    void Tracking::addPoints(const std::vector<cv::Point>& points, const cv::Mat& initFrame)
    {
        for(const auto& point : points)
        {
            m_multiTracker.push_back(Tracker{cv::TrackerKCF::create(),
                              /*.ROI =*/     cv::Rect2d(cv::Point2d(point.x - m_roiSize / 2., point.y - m_roiSize / 2.),
                                                         cv::Size2i(m_roiSize, m_roiSize))});
            m_multiTracker.back().tracker->init(initFrame, m_multiTracker.back().ROI);
        }
    }

    std::vector<cv::Point> Tracking::update(const cv::Mat& frame)
    {
        std::vector<cv::Point> points{};

        for(auto& tracker : m_multiTracker)
        {
            if(tracker.tracker->update(frame, tracker.ROI))
            {
                points.emplace_back(tracker.point());
            }
            else //Tracking is lost
            {
                points.clear();
                break;
            }
        }

        return points;
    }

    void Tracking::showTrackedPoint(const cv::Mat& frame)
    {
        for(const auto& tracker : m_multiTracker)
        {
            cv::rectangle(frame, tracker.ROI, cv::Scalar(0, 0, 255), 2);
            cv::circle(frame, tracker.point(), 2, cv::Scalar(0, 0, 255), -1);
        }

        imshow("Tracked points", frame);
    }
}

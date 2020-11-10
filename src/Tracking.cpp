//
// Created by Simon on 06/11/2020.
//

#include <opencv2/highgui.hpp>
#include <cmath>
#include "../headers/Tracking.hpp"
#include "../headers/Utils.hpp"

namespace arfs
{
    Tracking::Tracking(const cv::Mat& frame)
    {
        cv::Mat gray_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(gray_frame, m_corners, m_nbTrackersMax, 0.05, 4, cv::Mat(), 7, false, 0.04);
        for(const auto& c : m_corners)
        {
            m_multiTracker.push_back(Tracker{/*.tracker =*/ cv::TrackerMOSSE::create(),
                    /*.ROI =*/ cv::Rect2d(cv::Point2d(c.x - m_roiSize / 2., c.y - m_roiSize / 2.), cv::Size2i(m_roiSize, m_roiSize))});
            m_multiTracker.back().tracker->init(frame, m_multiTracker.back().ROI);
        }

        m_trackedPoints = std::vector<TrackedPoint>(m_corners.size());
    }

    void Tracking::update(const cv::Mat& frame)
    {
        for(int i = 0; i < m_multiTracker.size(); i++)
        {
            cv::Point2d lastPos = m_multiTracker[i].findCenter();
            if(!m_multiTracker[i].tracker->update(frame, m_multiTracker[i].ROI))
            {
                m_trackedPoints[i].isTracked = false;
            }
            else
            {
                cv::Point2d currentPos = m_multiTracker[i].findCenter();
                m_trackedPoints[i].lastPosition = currentPos;
                m_trackedPoints[i].totalDistance += cv::norm(lastPos - currentPos);
                m_trackedPoints[i].lastDistance = cv::norm(lastPos - currentPos);
                m_trackedPoints[i].lastAngle = Utils::angleBetween(currentPos, lastPos);
            }
        }
    }

    cv::Vec2d Tracking::getAvgMovement()
    {
        double anglesSum = 0;
        double distSum = 0;
        unsigned int nbPoints = 0;
        auto avgMovement = cv::Vec2d(0,0);

        for(const auto& p : m_trackedPoints)
        {
            if(p.isTracked)
            {
                distSum += p.lastDistance;
                anglesSum += p.lastAngle;
                nbPoints++;
            }
        }

        if(nbPoints > 0)
        {
            double angleRad = anglesSum / nbPoints;
            double avgDist = int(distSum / nbPoints);
            auto P1Arrow = cv::Point2d(100, 100);
            auto P2Arrow = cv::Point2d(P1Arrow.x + avgDist * std::cos(angleRad), P1Arrow.y + avgDist * std::sin(angleRad));
            avgMovement = cv::Vec2d(P2Arrow - P1Arrow);
        }

        return avgMovement;
    }

    void Tracking::showTrackedPoint(const cv::Mat& frame)
    {
        cv::Mat lastFrame = frame.clone();

        double anglesSum = 0;
        double distSum = 0;
        unsigned int nbPoints = 0;

        for(const auto& p : m_trackedPoints)
        {
            if(p.isTracked)
            {
                distSum += p.lastDistance;
                anglesSum += p.lastAngle;
                nbPoints++;
            }
        }

        if(nbPoints > 0)
        {
            double angleRad = anglesSum / nbPoints;
            double avgDist = distSum / nbPoints;
            std::cout << angleRad * (180.0 / Utils::PI) << " deg / " << angleRad << " rad" << std::endl;
            auto P1Arrow = cv::Point2d(100, 100);
            unsigned int length = int(avgDist) * 50;
            auto P2Arrow = cv::Point2d(P1Arrow.x + length * std::cos(angleRad), P1Arrow.y + length * std::sin(angleRad));
            cv::arrowedLine(lastFrame, P1Arrow, P2Arrow, cv::Scalar(0, 0, 255), 2, 1);
        }

        auto sortTracked = std::vector<TrackedPoint>(m_trackedPoints);
        std::sort(sortTracked.begin(), sortTracked.end(), [](const TrackedPoint& p1, const TrackedPoint& p2)
        { return p1.totalDistance < p2.totalDistance; });

        //TODO: if the last point is not tracked, it may be a problem
        double distanceMax = sortTracked.back().totalDistance;

        for(const auto& trackedPoint : sortTracked)
        {
            if(trackedPoint.isTracked)
            {
                //Todo: by using linear interpolation, any distance from the camera can be found
                unsigned int gray = int((trackedPoint.totalDistance / distanceMax) * 255);
                cv::circle(lastFrame, trackedPoint.lastPosition, m_roiSize / 4, cv::Scalar(gray, gray, gray), -1, 1);
            }
        }

        imshow("depth", lastFrame);
    }
}

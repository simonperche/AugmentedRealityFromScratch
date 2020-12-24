//
// Created by Simon on 22/12/2020.
//

#include <opencv2/imgproc.hpp>
#include <numeric>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "../headers/TagDetection.hpp"
#include "../headers/Utils.hpp"

namespace arfs
{
    std::vector<cv::Point> TagDetection::update(const cv::Mat& frame)
    {
        if(m_tagCorners.empty())
        {
            fullDetection(frame);
        }
        else
        {
            auto candidate = m_tracking.update(frame);

            bool enoughMovement = false;
            for(int i = 0 ; i < candidate.size() ; ++i)
            {
                if(arfs::Utils::norm(candidate[i], m_tagCorners[i]) > m_minimumDistance)
                {
                    enoughMovement = true;
                    break;
                }
            }

            if(!enoughMovement)
                candidate = m_tagCorners;

            if(candidate.empty() || !recognizeTag(frame, candidate))
                fullDetection(frame);
            else if(m_verbose)
                m_tracking.showTrackedPoint(frame);
        }

        return m_tagCorners;
    }

    void TagDetection::fullDetection(const cv::Mat& frame)
    {
        auto tagCandidates = extractTagCandidates(frame);
        for(auto candidate : tagCandidates)
        {
            if(recognizeTag(frame, candidate))
            {
                m_tracking.clear();
                m_tracking.addPoints(m_tagCorners, frame);
                break;
            }
        }
    }

    std::vector<std::vector<cv::Point>> TagDetection::extractTagCandidates(const cv::Mat& frame)
    {
        std::vector<std::vector<cv::Point>> candidates{};

        cv::Mat thresh;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::adaptiveThreshold(gray, thresh, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 11, 12);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        auto frame_copy = frame.clone();

        for(const auto& contour : contours)
        {
            if(cv::contourArea(contour) <= 1000)
                continue;

            //std::vector<std::vector> is needed to use draw contours
            auto hull = std::vector<std::vector<cv::Point>>(1);
            cv::convexHull(contour, hull[0]);

            const double& angleThreshold = 5;
            const double& distanceThreshold = 20;

            //Remove aligned and merge close points
            for(size_t i = 0; i < hull[0].size(); i++)
            {
                if(hull[0].size() < 4)
                    break;
                const auto& p = hull[0][i];
                const auto& previous = (i != 0) ? hull[0][i - 1] : hull[0][hull[0].size() - 1];
                const auto& next = (i != hull[0].size() - 1) ? hull[0][i + 1] : hull[0][0];

                const auto& vec1 = next - p;
                const auto& vec2 = p - previous;

                double angle = arfs::Utils::angleBetween(vec1, vec2, arfs::AngleType::DEG);

                //Points are aligned
                if(angle < angleThreshold)
                {
                    hull[0].erase(hull[0].begin() + i);
                    i--;
                }
            }

            std::vector<std::vector<cv::Point>> close_points{};

            auto groupClosePoints = [](std::vector<std::vector<cv::Point>>& groups, const cv::Point& p1,
                                       const cv::Point& p2)
            {
                bool found = false;
                for(auto& group : groups)
                {
                    if(std::find(group.begin(), group.end(), p1) != group.end())
                    {
                        group.push_back(p2);
                        found = true;
                    }
                    else if(std::find(group.begin(), group.end(), p2) != group.end())
                    {
                        group.push_back(p1);
                        found = true;
                    }
                }

                if(!found)
                    groups.push_back(std::vector<cv::Point>{p1, p2});
            };

            //Need another loop because vector change over iteration on the previous loop
            for(size_t i = 0; i < hull[0].size(); i++)
            {
                const auto& p = hull[0][i];
                const auto& previous = (i != 0) ? hull[0][i - 1] : hull[0][hull[0].size() - 1];
                const auto& next = (i != hull[0].size() - 1) ? hull[0][i + 1] : hull[0][0];

                if(arfs::Utils::norm(p, previous) < distanceThreshold)
                    groupClosePoints(close_points, p, previous);
                else if(arfs::Utils::norm(p, next) < distanceThreshold)
                    groupClosePoints(close_points, p, next);
                else
                    close_points.push_back(std::vector<cv::Point>{p});

            }

            std::vector<cv::Point> points;
            for(const auto& group : close_points)
            {
                auto newPoint = std::accumulate(group.begin(), group.end(), cv::Point(0, 0)) / (int) group.size();
                points.push_back(newPoint);
            }

            if(points.size() != 4)
                continue;

            candidates.push_back(points);
        }

        return candidates;
    }

    bool TagDetection::recognizeTag(const cv::Mat& frame, std::vector<cv::Point>& candidate)
    {
        m_tagCorners.clear();

        auto dstPoints = std::vector<cv::Point>{cv::Point(0, 0),
                                                cv::Point(300, 0),
                                                cv::Point(300, 300),
                                                cv::Point(0, 300)};

        bool found = false;

        //Try all rotations, homography is sensible to order of points
        for(int rotation = 0; rotation < 4; rotation++)
        {
            std::rotate(candidate.begin(), candidate.begin() + 1, candidate.end());
            cv::Mat homography = arfs::Utils::estimateHomography(candidate, dstPoints);
            if(homography.empty()) continue;

            cv::Mat candidate_mat = arfs::Utils::wrapPerspective(frame, cv::Size(300, 300), homography);
            cv::cvtColor(candidate_mat, candidate_mat, cv::COLOR_BGR2GRAY);
            cv::threshold(candidate_mat, candidate_mat, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

            auto candidateTag = arfs::ARTag(candidate_mat);
            if(candidateTag.getCode() == m_tagToDetect.getCode())
            {
                found = true;
                m_tagCorners = candidate;

                if(m_verbose)
                    cv::imshow("candidate", candidate_mat);

                break;
            }
        }

        return found;
    }
}
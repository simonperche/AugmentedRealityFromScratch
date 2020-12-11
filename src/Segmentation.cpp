//
// Created by Simon on 10/11/2020.
//

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cmath>
#include <numeric>
#include "../headers/Segmentation.hpp"
#include "../headers/Utils.hpp"

namespace arfs
{

    cv::Mat
    Segmentation::segmentation(const cv::Mat& img, const std::vector<std::pair<cv::Point2d, double>>& depthPoints)
    {
//        cv::Mat grayFrame;
//        cv::cvtColor(img, grayFrame, cv::COLOR_BGR2GRAY);
//
//        for(const auto& p : depthPoints)
//        {
//            cv::circle(grayFrame, p.first, 5, cv::Scalar(p.second, p.second, p.second), -1, 1);
//        }
//
//        cv::imshow("gray", grayFrame);

//Prepare the image for findContours
        cv::Mat image = img.clone();
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
        cv::threshold(image, image, 128, 255, cv::THRESH_BINARY);

        //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
        std::vector<std::vector<cv::Point> > contours;
        cv::Mat contourOutput = image.clone();
        cv::findContours(contourOutput, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

        //Draw the contours
        cv::Mat contourImage(image.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);
        for(size_t idx = 0; idx < contours.size(); idx++)
        {
            cv::drawContours(contourImage, contours, idx, colors[idx % 3]);
        }

        cv::imshow("Input Image", image);
        cv::imshow("Contours", contourImage);

        cv::waitKey();

        return cv::Mat();
    }

    cv::Mat Segmentation::extractTag(const cv::Mat& frame)
    {
        auto thresh = threshold(frame);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        auto rng = cv::RNG(12345);
        auto drawing = cv::Mat(thresh.size(), CV_8UC3, cv::Scalar(0));

        auto frame_copy = frame.clone();

        for(size_t i = 0; i < contours.size(); i++)
        {
            if(cv::contourArea(contours[i]) <= 1000)
                continue;

            cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            cv::drawContours(drawing, contours, (int) i, color, 2, cv::LINE_8, hierarchy, 0);

            auto hull_mask = cv::Mat(frame.size(), CV_8UC1, cv::Scalar(0));

            //std::vector<std::vector> is needed to use draw contours
            auto hull = std::vector<std::vector<cv::Point>>(1);
            cv::convexHull(contours[i], hull[0]);
            cv::drawContours(hull_mask, hull, 0, cv::Scalar(255), 1, cv::LINE_8);
//            cv::fillPoly(hull_mask, hull, cv::Scalar(255));

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

            for(const auto& p : points)
            {
                cv::circle(hull_mask, p, 5, cv::Scalar(128), cv::FILLED);
                for(size_t i = 0 ; i < points.size()-1;i++)
                    cv::line( frame_copy, points[i], points[i+1], cv::Scalar(0,255,0), 1, cv::LINE_AA);
                cv::line( frame_copy, points[3], points[0], cv::Scalar(0,255,0), 1, cv::LINE_AA);
                cv::circle(frame_copy, p, 2, cv::Scalar(0,0,255), cv::FILLED);
            }

            cv::imshow("hull", hull_mask);
            cv::Mat warpedSquare(300, 300, CV_8UC3);
            cv::Mat homography = cv::findHomography(points, std::vector<cv::Point>{cv::Point(0,0) , cv::Point(warpedSquare.cols, 0), cv::Point(warpedSquare.cols, warpedSquare.rows), cv::Point(0, warpedSquare.rows) });
            warpPerspective(frame, warpedSquare, homography, cv::Size(warpedSquare.cols, warpedSquare.rows));
            cv::imshow("Square", warpedSquare);
//            cv::waitKey();
        }
        cv::imshow("Contours", drawing);
        cv::imshow("Tag", frame_copy);

        return thresh;
    }

    cv::Mat Segmentation::threshold(const cv::Mat& img)
    {
        cv::Mat thresh;
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::adaptiveThreshold(gray, thresh, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 11, 12);
        return thresh;
    }
}
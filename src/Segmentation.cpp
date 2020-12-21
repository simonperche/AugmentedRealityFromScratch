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
#include <algorithm>
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

    std::vector<std::vector<cv::Point>> Segmentation::extractTagCandidates(const cv::Mat& frame)
    {
        std::vector<std::vector<cv::Point>> candidates{};
        auto thresh = threshold(frame);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

//        auto rng = cv::RNG(12345);
//        auto drawing = cv::Mat(thresh.size(), CV_8UC3, cv::Scalar(0));

        auto frame_copy = frame.clone();

        for(size_t i = 0; i < contours.size(); i++)
        {
            if(cv::contourArea(contours[i]) <= 1000)
                continue;

//            cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
//            cv::drawContours(drawing, contours, (int) i, color, 2, cv::LINE_8, hierarchy, 0);

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
                    } else if(std::find(group.begin(), group.end(), p2) != group.end())
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
//            for(const auto& p : points)
//            {
//                cv::circle(hull_mask, p, 5, cv::Scalar(128), cv::FILLED);
//                for(size_t i = 0 ; i < points.size()-1;i++)
//                    cv::line( frame_copy, points[i], points[i+1], cv::Scalar(0,255,0), 1, cv::LINE_AA);
//                cv::line( frame_copy, points[3], points[0], cv::Scalar(0,255,0), 1, cv::LINE_AA);
//                cv::circle(frame_copy, p, 2, cv::Scalar(0,0,255), cv::FILLED);
//            }

//            cv::imshow("hull", hull_mask);
//            cv::Mat warpedSquare(300, 300, CV_8UC3);
//            cv::Mat homography = cv::findHomography(points, std::vector<cv::Point>{cv::Point(0,0) , cv::Point(warpedSquare.cols, 0), cv::Point(warpedSquare.cols, warpedSquare.rows), cv::Point(0, warpedSquare.rows) });
//            warpPerspective(frame, warpedSquare, homography, cv::Size(warpedSquare.cols, warpedSquare.rows));
//            cv::imshow("Square", warpedSquare);
////            cv::waitKey();
        }
//        cv::imshow("Contours", drawing);
//        cv::imshow("Tag", frame_copy);

        return candidates;
    }

    cv::Mat Segmentation::threshold(const cv::Mat& img)
    {
        cv::Mat thresh;
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::adaptiveThreshold(gray, thresh, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 11, 12);
        return thresh;
    }

    std::vector<cv::Point>
    Segmentation::recognizeTag(const cv::Mat& frame, std::vector<std::vector<cv::Point>> candidates,
                               std::array<int, 64> code)
    {
        std::vector<cv::Point> tag{};
        for(auto points : candidates)
        {
            cv::Mat candidate(300, 300, CV_8UC3);
            bool found = false;

            //Try all rotations, homography is sensible to order of points
            for(int rotation = 0; rotation < 4; rotation++)
            {
                std::rotate(points.begin(), points.begin() + 1, points.end());
                cv::Mat homography = cv::findHomography(points, std::vector<cv::Point>{cv::Point(0, 0),
                                                                                       cv::Point(candidate.cols, 0),
                                                                                       cv::Point(candidate.cols,
                                                                                                 candidate.rows),
                                                                                       cv::Point(0, candidate.rows)});
                cv::warpPerspective(frame, candidate, homography, cv::Size(candidate.cols, candidate.rows));

                cv::cvtColor(candidate, candidate, cv::COLOR_BGR2GRAY);

                cv::threshold(candidate, candidate, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);


                if(getARTagCode(candidate) == code)
                {
                    found = true;
                    cv::imshow("candidate", candidate);
                    tag = points;
                    auto f = frame.clone();
                    showAxis(homography, tag, f);
                    break;
                }
            }

            if(found) break;
        }

        return tag;
    }

    std::array<int, 64> Segmentation::getARTagCode(const cv::Mat& tag_img)
    {
        // 8 is fixed because aruco tags work with 8x8 square
        unsigned int width = tag_img.cols / 8;
        std::array<int, 64> code{};

        for(unsigned int i = 0; i < 8; i++)
        {
            for(unsigned int j = 0; j < 8; j++)
            {
                auto rect = cv::Rect(j * width, i * width, width, width);
                cv::Scalar mean = cv::mean(tag_img(rect));
                code[j + i * 8] = (mean[0] < 128) ? 0 : 1;
            }
        }

        return code;
    }

    void Segmentation::showAxis(const cv::Mat& homography, const std::vector<cv::Point>& tag, cv::Mat& frame)
    {
        auto intrinsic = cv::Matx33d(28, 0, frame.cols / 2.,
                             0, 28, frame.rows / 2.,
                             0, 0, 1);

        auto obj_points = std::vector<cv::Point3d>{cv::Point3d(150, 150, 0),
                                                   cv::Point3d(150, 250, 0),
                                                   cv::Point3d(250, 150, 0),
                                                   cv::Point3d(150, 150, 100)};

        auto scene_points = projectPoint(obj_points, getProjectionMatrix(homography, intrinsic));
        std::cout << "NEW" << std::endl;
        std::cout << scene_points << std::endl;
        std::cout << tag << std::endl;

        cv::line(frame, scene_points[0], scene_points[1], cv::Scalar(0, 0, 255), 3);
        cv::line(frame, scene_points[0], scene_points[2], cv::Scalar(0, 255, 0), 3);
        cv::line(frame, scene_points[0], scene_points[3], cv::Scalar(255, 0, 0), 3);
        cv::imshow("axis", frame);
    }

    std::vector<cv::Point2d> Segmentation::projectPoint(const std::vector<cv::Point3d>& points, const cv::Mat& projectionMatrix)
    {
        std::vector<cv::Point2d> scene_points(points.size());
        for(int i = 0; i < points.size(); i++)
        {
            cv::Mat_<double> src(4, 1);

            src(0, 0) = points[i].x;
            src(1, 0) = points[i].y;
            src(2, 0) = points[i].z;
            src(3, 0) = 1;
            cv::Mat tmp = projectionMatrix * src;
            tmp /= tmp.at<double>(2, 0);
            scene_points[i].x = tmp.at<double>(0, 0);
            scene_points[i].y = tmp.at<double>(1, 0);

        }

        return scene_points;
    }

    cv::Mat Segmentation::getProjectionMatrix(const cv::Mat& homography, const cv::Matx33d& intrinsicMatrix)
    {
        cv::Mat homography_inv = homography.inv();

        auto R_and_T = intrinsicMatrix.inv() * homography_inv;
        auto r1 = R_and_T.col(0);
        auto r2 = R_and_T.col(1);
        auto T = R_and_T.col(2);
        auto norm = sqrt(cv::norm(r1) * cv::norm(r2));

        r1 /= norm;
        r2 /= norm;
        T /= norm;
        auto c = r1 + r2;
        auto p = r1.cross(r2);
        auto d = c.cross(p);
        r1 = (c / cv::norm(c) + d / cv::norm(d)) * (1 / sqrt(2));
        r2 = (c / cv::norm(c) - d / cv::norm(d)) * (1 / sqrt(2));
        auto r3 = r1.cross(r2);

        cv::Mat R_T;
        cv::Mat array[] = {r1, r2, r3, T};
        cv::hconcat(array, 4, R_T);

        return intrinsicMatrix * R_T;
    }
}
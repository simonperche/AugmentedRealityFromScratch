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
#include "../headers/OBJLoader.h"

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
            cv::drawContours(contourImage, contours, int(idx), colors[idx % 3]);
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

//            auto hull_mask = cv::Mat(frame.size(), CV_8UC1, cv::Scalar(0));

            //std::vector<std::vector> is needed to use draw contours
            auto hull = std::vector<std::vector<cv::Point>>(1);
            cv::convexHull(contours[i], hull[0]);
//            cv::drawContours(hull_mask, hull, 0, cv::Scalar(255), 1, cv::LINE_8);
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
        }

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
            cv::Mat candidate(m_tagSize, m_tagSize, CV_8UC3);
            bool found = false;

            //Try all rotations, homography is sensible to order of points
            for(int rotation = 0; rotation < 4; rotation++)
            {
                std::rotate(points.begin(), points.begin() + 1, points.end());
                cv::Mat homography = computeHomography(points);
                if(homography.empty()) continue;

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
        auto intrinsic = cv::Matx33d(576.1357414740778, 0, 311.1655482832962,
                                    0, 573.553686984999, 233.570912689823,
                                    0, 0, 1);
//        auto intrinsic = cv::Matx33d(3.6, 0, frame.cols / 2.,
//                                     0, 3.6, frame.rows / 2.,
//                                     0, 0, 1);
        auto projectionMatrix = getProjectionMatrix(homography, intrinsic);

        auto obj_points = std::vector<cv::Point3d>{cv::Point3d(m_tagSize/2, m_tagSize/2, 0),
                                                   cv::Point3d(m_tagSize/2, (m_tagSize/2)+100, 0),
                                                   cv::Point3d((m_tagSize/2)+100, m_tagSize/2, 0),
                                                   cv::Point3d(m_tagSize/2, m_tagSize/2, 100)};

        auto scene_points = projectPoint(obj_points, projectionMatrix);
//        std::cout << std::endl << "AXIS" << std::endl;
//        std::cout << homography << std::endl;
//        std::cout << scene_points << std::endl;
//        std::cout << tag << std::endl;

        cv::line(frame, scene_points[0], scene_points[1], cv::Scalar(0, 0, 255), 3);
        cv::line(frame, scene_points[0], scene_points[2], cv::Scalar(0, 255, 0), 3);
        cv::line(frame, scene_points[0], scene_points[3], cv::Scalar(255, 0, 0), 3);
        cv::imshow("axis", frame);
    }

    std::vector<cv::Point2i> Segmentation::projectPoint(const std::vector<cv::Point3d>& points, const cv::Mat& projectionMatrix)
    {
        std::vector<cv::Point2i> scene_points(points.size());
        for(int i = 0; i < points.size(); i++)
        {
            cv::Mat_<double> src(4, 1);

            src(0, 0) = points[i].x;
            src(1, 0) = points[i].y;
            src(2, 0) = points[i].z;
            src(3, 0) = 1;
            cv::Mat tmp = projectionMatrix * src;
            tmp /= tmp.at<double>(2, 0);
            scene_points[i].x = int(tmp.at<double>(0, 0));
            scene_points[i].y = int(tmp.at<double>(1, 0));

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

    void Segmentation::augmentObject(const OBJLoader& obj, const cv::Mat& frame, const std::vector<cv::Point>& tag)
    {
        cv::Mat homography = computeHomography(tag);

        if(homography.empty()) return;

        auto intrinsic = cv::Matx33d(576.1357414740778, 0, 311.1655482832962,
                                     0, 573.553686984999, 233.570912689823,
                                     0, 0, 1);
        auto projectionMatrix = getProjectionMatrix(homography, intrinsic);
        cv::Mat translation = (intrinsic.inv() * projectionMatrix).col(3);

        auto faces = obj.getFaces();

        std::sort(faces.begin(), faces.end(), [&](Face a, Face b)
        {
            auto a_mean = cv::Point3d(0,0,0);
            auto b_mean = cv::Point3d(0,0,0);
            for(const auto& point : a.points)
                a_mean += point;
            for(const auto& point : b.points)
                b_mean += point;
            a_mean /= (double)a.points.size();
            b_mean /= (double)b.points.size();
            return cv::norm(cv::Point3d(translation) - a_mean) < cv::norm(cv::Point3d(translation) - b_mean);
        });
        for(auto& face : faces)
        {
            for(auto& point : face.points)
            {
                //Scale
                point *= 3;

                //Center
                point.x += int(m_tagSize/2);
                point.y += int(m_tagSize/2);
            }
            auto scene_points = projectPoint(face.points, projectionMatrix);

            auto light = cv::Vec3d(-1,1,-1);
            auto angle = arfs::Utils::angleBetween(face.normal, light, arfs::AngleType::DEG);
            auto lightValue = (angle*255)/180;
            cv::fillConvexPoly(frame, scene_points, cv::Scalar(lightValue,lightValue,lightValue));
//            std::cout << "FACE" << std::endl;
//            std::cout << homography << std::endl;
//            std::cout << face << std::endl;
//            std::cout << scene_points << std::endl;

            cv::imshow("object", frame);
        }
    }

    cv::Mat Segmentation::computeHomography(const std::vector<cv::Point>& srcPoints)
    {
        if(srcPoints.size() != 4) return cv::Mat();

        const int factor = 6;
        auto dstPoints = std::vector<cv::Point>{cv::Point(0, 0),
                                                cv::Point(m_tagSize, 0),
                                                cv::Point(m_tagSize, m_tagSize),
                                                cv::Point(0, m_tagSize)};

        // -- This was a test trying to augment the number of point to compute a better homography
//        std::vector<cv::Point2d> srcPointsAugmented{};
//        std::vector<cv::Point> dstPointsAugmented{};
//
//        auto getPourcentageOfSegment = [](const cv::Point& p1, const cv::Point& p2, double i, double factor)
//        {
//            return static_cast<cv::Point2d>(p1) + (i/factor)*(static_cast<cv::Point2d>(p2) - static_cast<cv::Point2d>(p1));
//        };
//
//        //Divide polygon in a grid to augment the number of points
//        for(int i = 0 ; i <= factor ; i++)
//        {
//            cv::Point2d p1_src = getPourcentageOfSegment(srcPoints[0], srcPoints[1], i, factor);
//            cv::Point2d p2_src = getPourcentageOfSegment(srcPoints[3], srcPoints[2], i, factor);
//
//            cv::Point2d p1_dst = getPourcentageOfSegment(dstPoints[0], dstPoints[1], i, factor);
//            cv::Point2d p2_dst = getPourcentageOfSegment(dstPoints[3], dstPoints[2], i, factor);
//            for(int j = 0 ; j <= factor ; j++)
//            {
//                cv::Point2d p_src = getPourcentageOfSegment(p1_src, p2_src, j, factor);
//                srcPointsAugmented.emplace_back(p_src);
//
//                cv::Point2d p_dst = getPourcentageOfSegment(p1_dst, p2_dst, j, factor);
//                dstPointsAugmented.emplace_back(p_dst);
//            }
//        }
//
//        cv::Mat homography = cv::findHomography(srcPointsAugmented, dstPointsAugmented);

        cv::Mat homography = cv::findHomography(srcPoints, dstPoints);

        return homography;
    }
}
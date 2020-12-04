//
// Created by Simon on 10/11/2020.
//

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include "../headers/Segmentation.hpp"

namespace arfs
{

    cv::Mat Segmentation::segmentation(const cv::Mat& img, const std::vector<std::pair<cv::Point2d, double>>& depthPoints)
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
        cv::findContours( contourOutput, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE );

        //Draw the contours
        cv::Mat contourImage(image.size(), CV_8UC3, cv::Scalar(0,0,0));
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);
        for (size_t idx = 0; idx < contours.size(); idx++) {
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
        cv::Mat drawing = cv::Mat::zeros( thresh.size(), CV_8UC3 );
        for( size_t i = 0; i< contours.size(); i++ )
        {
            if(cv::contourArea(contours[i]) > 1000)
            {
                cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
                drawContours( drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
            }
        }
        cv::imshow( "Contours", drawing );

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
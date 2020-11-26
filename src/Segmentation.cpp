//
// Created by Simon on 10/11/2020.
//

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
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
}
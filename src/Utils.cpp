//
// Created by Simon on 09/11/2020.
//

#include <cmath>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "../headers/Utils.hpp"

namespace arfs
{

    double Utils::angleBetween(const cv::Point& p1, const cv::Point& p2, AngleType type)
    {
        double angle = acos(p1.dot(p2) / (cv::norm(p1) * cv::norm(p2)));
        if(type == AngleType::DEG)
            angle = angle * 180 / CV_PI;

        return angle;
    }

    double Utils::angleBetween(const cv::Vec3d& v1, const cv::Vec3d& v2, AngleType type)
    {
        double angle = acos(v1.dot(v2) / (cv::norm(v1) * cv::norm(v2)));
        if(type == AngleType::DEG)
            angle = angle * 180 / CV_PI;

        return angle;
    }

    double Utils::norm(const cv::Point& p1, const cv::Point& p2)
    {
        return cv::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    void Utils::saveImage(const cv::Mat& img, const std::string& filename)
    {
        cv::imwrite(filename, img);
    }

    void Utils::showImage(const cv::Mat& img, const std::string& winname)
    {
        cv::imshow(winname, img);
    }

    cv::Mat Utils::loadImage(const std::string& filename)
    {
        return cv::imread(filename);
    }

    std::vector<std::string> Utils::split(std::string s, const char& delimiter)
    {
        std::vector<std::string> split_strings{};
        size_t pos;
        std::string token{};
        while((pos = s.find(delimiter)) != std::string::npos)
        {
            token = s.substr(0, pos);
            split_strings.emplace_back(token);
            s.erase(0, pos + 1);
        }
        split_strings.emplace_back(s);

        return split_strings;
    }

    cv::Mat Utils::wrapPerspective(const cv::Mat& src, const cv::Size& size, const cv::Mat& matrix)
    {
        auto dst = cv::Mat(size, CV_8UC3);
        cv::Mat matrixInv = matrix.inv();
        std::vector<cv::Mat> channels_src(3);
        std::vector<cv::Mat> channels_dst(3);
        cv::split(src, channels_src);
        cv::split(dst, channels_dst);

        for(int j = 0; j < dst.rows; ++j)
        {
            for(int i = 0; i < dst.cols; ++i)
            {
                int x = int((matrixInv.at<double>(0, 0) * j + matrixInv.at<double>(0, 1) * i + matrixInv.at<double>(0, 2))
                        / (matrixInv.at<double>(2,0) * j + matrixInv.at<double>(2, 1) * i + matrixInv.at<double>(2, 2)));
                int y = int((matrixInv.at<double>(1, 0) * j + matrixInv.at<double>(1, 1) * i + matrixInv.at<double>(1, 2))
                        / (matrixInv.at<double>(2,0) * j + matrixInv.at<double>(2, 1) * i + matrixInv.at<double>(2, 2)));
                channels_dst[0].at<unsigned char>(i, j) = channels_src[0].at<unsigned char>(y, x);
                channels_dst[1].at<unsigned char>(i, j) = channels_src[1].at<unsigned char>(y, x);
                channels_dst[2].at<unsigned char>(i, j) = channels_src[2].at<unsigned char>(y, x);
            }
        }

        cv::merge(channels_dst, dst);

        return dst;
    }

    cv::Mat Utils::estimateHomography(const std::vector<cv::Point>& srcPoints, const std::vector<cv::Point>& dstPoints)
    {
        cv::Mat matrix = cv::Mat(3, 3, CV_64F);
        std::array<double, 72> data{double(-srcPoints[0].x), double(-srcPoints[0].y), -1.0, 0.0, 0.0, 0.0,
                                    double(srcPoints[0].x) * double(dstPoints[0].x), double(srcPoints[0].y) * double(dstPoints[0].x),
                                    double(-dstPoints[0].x),
                                    0.0, 0.0, 0.0, double(-srcPoints[0].x), double(-srcPoints[0].y), -1.0,
                                    double(srcPoints[0].x) * double(dstPoints[0].y), double(srcPoints[0].y) * double(dstPoints[0].y),
                                    double(-dstPoints[0].y),
                                    double(-srcPoints[1].x), double(-srcPoints[1].y), -1.0, 0.0, 0.0, 0.0,
                                    double(srcPoints[1].x) * double(dstPoints[1].x), double(srcPoints[1].y) * double(dstPoints[1].x),
                                    double(-dstPoints[1].x),
                                    0.0, 0.0, 0.0, double(-srcPoints[1].x), double(-srcPoints[1].y), -1.0,
                                    double(srcPoints[1].x) * double(dstPoints[1].y), double(srcPoints[1].y) * double(dstPoints[1].y),
                                    double(-dstPoints[1].y),
                                    double(-srcPoints[2].x), double(-srcPoints[2].y), -1.0, 0.0, 0.0, 0.0,
                                    double(srcPoints[2].x) * double(dstPoints[2].x), double(srcPoints[2].y) * double(dstPoints[2].x),
                                    double(-dstPoints[2].x),
                                    0.0, 0.0, 0.0, double(-srcPoints[2].x), double(-srcPoints[2].y), -1.0,
                                    double(srcPoints[2].x) * double(dstPoints[2].y), double(srcPoints[2].y) * double(dstPoints[2].y),
                                    double(-dstPoints[2].y),
                                    double(-srcPoints[3].x), double(-srcPoints[3].y), -1.0, 0.0, 0.0, 0.0,
                                    double(srcPoints[3].x) * double(dstPoints[3].x), double(srcPoints[3].y) * double(dstPoints[3].x),
                                    double(-dstPoints[3].x),
                                    0.0, 0.0, 0.0, double(-srcPoints[3].x), double(-srcPoints[3].y), -1.0,
                                    double(srcPoints[3].x) * double(dstPoints[3].y), double(srcPoints[3].y) * double(dstPoints[3].y),
                                    double(-dstPoints[3].y),};

        cv::Mat homographyMatrix = cv::Mat(8, 9, CV_64F, data.data());

        gaussJordanElimination(homographyMatrix, 8, 9);

        matrix.at<double>(0, 0) = homographyMatrix.at<double>(0, 8);
        matrix.at<double>(0, 1) = homographyMatrix.at<double>(1, 8);
        matrix.at<double>(0, 2) = homographyMatrix.at<double>(2, 8);
        matrix.at<double>(1, 0) = homographyMatrix.at<double>(3, 8);
        matrix.at<double>(1, 1) = homographyMatrix.at<double>(4, 8);
        matrix.at<double>(1, 2) = homographyMatrix.at<double>(5, 8);
        matrix.at<double>(2, 0) = homographyMatrix.at<double>(6, 8);
        matrix.at<double>(2, 1) = homographyMatrix.at<double>(7, 8);
        matrix.at<double>(2, 2) = 1;

        return matrix;
    }


    void Utils::gaussJordanElimination(cv::Mat& matrix, int rows, int cols)
    {
        //Pivot initialization
        int h = 0;
        int k = 0;

        while(h < rows && k < cols)
        {
            /* Find the k-th pivot */
            int i_max = h;
            for(int i = h + 1; i < rows; i++)
            {
                if(fabs(matrix.at<double>(i, k)) > fabs(matrix.at<double>(i_max, k)))
                {
                    i_max = i;
                }
            }
            if(matrix.at<double>(i_max, k) == 0)
            {
                // No pivot, jump this column
                k = k + 1;
            }
            else
            {
                if(i_max != h)
                {
                    for(int l = 0; l < cols; l++)
                    {
                        double save = matrix.at<double>(h, l);
                        matrix.at<double>(h, l) = matrix.at<double>(i_max, l);
                        matrix.at<double>(i_max, l) = save;
                    }
                }

                double norm = matrix.at<double>(h, k);
                for(int l = 0; l < cols; l++)
                {
                    matrix.at<double>(h, l) /= norm;
                }

                for(int i = h + 1; i < rows; i++)
                {
                    double f = matrix.at<double>(i, k) / matrix.at<double>(h, k);
                    for(int j = k + 1; j < cols; j++)
                    {
                        matrix.at<double>(i, j) -= (matrix.at<double>(h, j)) * f;
                    }
                }

                h = h + 1;
                k = k + 1;
            }

        }

        //Back substitution, to transform the matrix in row echelon form in a system
        for(int i = rows - 2; i >= 0; i--)
        {
            for(int j = i + 1; j < cols - 1; j++)
            {
                matrix.at<double>(i, rows) -= matrix.at<double>(i, j) * matrix.at<double>(j, rows);
            }
        }
    }
}
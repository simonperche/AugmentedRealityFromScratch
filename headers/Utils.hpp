//
// Created by Simon on 09/11/2020.
//

#ifndef AR_UTILS_HPP
#define AR_UTILS_HPP

#include <opencv2/core/types.hpp>

namespace arfs
{
    enum AngleType
    {
        DEG,
        RAD
    };

    namespace Utils
    {
        std::vector<std::string> split(std::string s, const char& delimiter);

        namespace Geometry
        {
            constexpr double PI = 3.141592653589793238463;

            double angleBetween(const cv::Point& v1, const cv::Point& v2, AngleType type = AngleType::RAD);

            double angleBetween(const cv::Vec3d& v1, const cv::Vec3d& v2, AngleType type = AngleType::RAD);

            // cv::norm is not implemented with two points
            double norm(const cv::Point& p1, const cv::Point& p2);

            inline double degToRad(double deg)
            { return deg * (PI / 180); }
        }

        namespace Image
        {
            void saveImage(const cv::Mat& img, const std::string& filename);

            void showImage(const cv::Mat& img, const std::string& winname);

            cv::Mat loadImage(const std::string& filename);
        }

        namespace CV
        {
            /**
            * Estimate the homography matrix between two sets of four points
            * @return homography 3x3 matrix
            * @param srcPoints
            * @param dstPoints
            */
            cv::Mat estimateHomography(const std::vector<cv::Point>& srcPoints, const std::vector<cv::Point>& dstPoints);

            cv::Mat wrapPerspective(const cv::Mat& src, const cv::Size& size, const cv::Mat& matrix);

            /**
            * Apply a gaussian elimination on a matrix
            * @param matrix matrix to solve
            * @param rows rows count on matrix
            * @param cols cols count on matrix
            */
            static void gaussJordanElimination(cv::Mat& matrix, int rows, int cols);
        }
    }
}

#endif //AR_UTILS_HPP

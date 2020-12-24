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

    class Utils
    {
    public:
        static double angleBetween(const cv::Point& v1, const cv::Point& v2, AngleType type = AngleType::RAD);
        static double angleBetween(const cv::Vec3d& v1, const cv::Vec3d& v2, AngleType type = AngleType::RAD);

        static double degToRad(double deg)
        { return deg * (Utils::PI / 180); }

        // cv::norm is not implemented with two points
        static double norm(const cv::Point& p1, const cv::Point& p2);

        static void saveImage(const cv::Mat& img, const std::string& filename);

        static void showImage(const cv::Mat& img, const std::string& winname);

        static cv::Mat loadImage(const std::string& filename);

        static std::vector<std::string> split(std::string s, const char& delimiter);

        /**
         * Estimate the homography matrix between two sets of four points
         * @param srcPoints
         * @param dstPoints
         * @param matrix output matrix
         */
        static void estimateHomography(const std::vector<cv::Point>& srcPoints, const std::vector<cv::Point>& dstPoints, cv::Mat matrix);

        static cv::Mat wrapPerspective(const cv::Mat& src, const cv::Size& size, const cv::Mat& matrix);

        static constexpr double PI = 3.141592653589793238463;

    private:
        /**
         * Apply a gaussian elimination on a matrix
         * @param matrix matrix to solve
         * @param rows rows count on matrix
         * @param cols cols count on matrix
         */
        static void gaussJordanElimination(cv::Mat &matrix, int rows, int cols);
    };
}

#endif //AR_UTILS_HPP

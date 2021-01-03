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
        /**
         * @brief Split a string using delimiter.
         * @param s original string
         * @param delimiter
         * @return array of string representing the original string divided by delimiters
         */
        std::vector<std::string> split(std::string s, const char& delimiter);

        /**
         * @brief Check if escape is pressed at this moment.
         * @return true if escape is pressed, false otherwise
         */
        bool escIsPressed();

        namespace Geometry
        {
            constexpr double PI = 3.141592653589793238463;

            /**
             * @brief Calculate angle between two points
             * @param p1 first point
             * @param p2 second point
             * @param type type of return angle (deg or rad)
             * @return the angle as type (deg or rad)
             */
            double angleBetween(const cv::Point& p1, const cv::Point& p2, AngleType type = AngleType::RAD);

            /**
             * @brief Calculate angle between 3D vectors
             * @param v1 first vector
             * @param v2 second vector
             * @param type type of return angle (deg or rad)
             * @return the angle as type (deg or rag)
             */
            double angleBetween(const cv::Vec3d& v1, const cv::Vec3d& v2, AngleType type = AngleType::RAD);

            /**
             * @brief Calculate the norm bewteen two points
             * @param p1 first point
             * @param p2 second point
             * @return norm
             */
            // cv::norm is not implemented with two points
            double norm(const cv::Point& p1, const cv::Point& p2);

            inline double degToRad(double deg)
            { return deg * (PI / 180); }
        }

        namespace Image
        {
            /**
             * @brief Save image as filename.
             * @param img
             * @param filename
             */
            void saveImage(const cv::Mat& img, const std::string& filename);

            /**
             * @brief Display image on screen.
             * @param img
             * @param winName
             */
            void showImage(const cv::Mat& img, const std::string& winName);

            /**
             * @brief Load an image from filename.
             * @param filename
             * @return Mat containing the image
             */
            cv::Mat loadImage(const std::string& filename);
        }

        namespace CV
        {
            /**
            * Estimate the homography matrix between two sets of points
            * @return homography 3x3 matrix
            * @param srcPoints
            * @param dstPoints
            */
            cv::Mat estimateHomography(const std::vector<cv::Point>& srcPoints, const std::vector<cv::Point>& dstPoints);

            /**
             * @brief Wrap an image using an homography matrix
             * @param src source image
             * @param size size of output image
             * @param matrix transformation matrix (usually homography)
             * @return image of indicated size, wrapped version of source image
             */
            cv::Mat wrapPerspective(const cv::Mat& src, const cv::Size& size, const cv::Mat& matrix);
        }
    }
}

#endif //AR_UTILS_HPP

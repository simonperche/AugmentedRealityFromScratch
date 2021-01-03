//
// Created by Simon on 22/12/2020.
//

#ifndef AR_CAMERA_HPP
#define AR_CAMERA_HPP

#include <opencv2/core/matx.hpp>
#include <string>
#include <opencv2/core/mat.hpp>
#include "Video.hpp"

namespace arfs
{
    /**
     * @brief Camera in the 3D scene containing intrinsic parameters.
     */
    class Camera
    {
    public:
        /**
         * @brief Load parameters from file.
         * @param filename .cam file created by calibration
         */
        void loadParameters(const std::string& filename);

        /**
         * @brief Calibrate the camera using checkerboard images.
         * @param filename where to save .cam file containing estimate parameters
         * @param checkerBoardSize the size of the checkerboard as array
         * @param imgFolder folder where to save or load images to do the calibration
         * @param resizeFactor resize image to accelerate calculation (default 1)
         * @param cap webcam to take pictures if needToTakePictures is true (default {})
         * @param needToTakePictures (default false)
         */
        void calibrateAndSave(const std::string& filename, const std::array<int, 2>& checkerBoardSize,
                              const std::string& imgFolder, double resizeFactor = 1,
                              arfs::Video cap = {}, bool needToTakePictures = false);

        /**
         * @brief Update the projection matrix using new tag points.
         * @details recalculate the homography from tag points and extract new projection matrix using intrinsic parameters
         * @param tagPoints tag detected in an image
         */
        void updateProjectionMatrix(const std::vector<cv::Point>& tagPoints);

        cv::Mat getProjectionMatrix() const
        { return m_projectionMatrix; }

        cv::Matx33d getIntrinsicParameters() const
        { return m_intrinsicParameters; }

        int getTagProjectionSize() const
        { return m_tagProjectionSize; }

    private:
        cv::Matx33d m_intrinsicParameters{};
        cv::Mat m_projectionMatrix{};
        const int m_tagProjectionSize{300};

        void saveParametersToFile(const std::string& filename);
    };
}


#endif //AR_CAMERA_HPP

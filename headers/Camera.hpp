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
    class Camera
    {
    public:
        void loadParameters(const std::string& filename);

        void calibrateAndSave(const std::string& filename, const std::array<int, 2>& checkerBoardSize,
                              const std::string& imgFolder, double resizeFactor = 1,
                              arfs::Video cap = {}, bool needToTakePictures = false);

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

        void estimateHomography(std::vector<cv::Point_<int>> tagPoints, std::vector<cv::Point_<int>> dstPoints, cv::Mat matrix);
        void gaussJordanElimination(cv::Mat &matrix, int rows, int cols);
    };
}


#endif //AR_CAMERA_HPP

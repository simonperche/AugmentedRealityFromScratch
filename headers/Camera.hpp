//
// Created by Simon on 22/12/2020.
//

#ifndef AR_CAMERA_HPP
#define AR_CAMERA_HPP

#include <opencv2/core/matx.hpp>
#include <string>
#include <opencv2/core/mat.hpp>

namespace arfs
{
    class Camera
    {
    public:
        Camera()
        {};

        void loadParameters(const std::string& filename);

        void calibrateAndSave(const std::string& filename);

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
        const int m_tagProjectionSize = 300;

        void saveParametersToFile(const std::string& filename);
    };
}


#endif //AR_CAMERA_HPP

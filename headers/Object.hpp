//
// Created by Simon on 22/12/2020.
//

#ifndef AR_OBJECT_HPP
#define AR_OBJECT_HPP

#include <vector>
#include <opencv2/core.hpp>

namespace arfs
{
    struct Face
    {
        std::vector<cv::Point3d> points;
        std::vector<cv::Point2i> textureCoordinate{};
        cv::Vec3d normal;
    };

    class Object
    {
    public:
        void addFace(const std::vector<cv::Point3d>& points, const std::vector<cv::Point2i>& textureCoordinate, const cv::Vec3d& normal = cv::Vec3d(0,0,0));
        void setTextureImage(const std::string& filename);
        void rotate(double xAngle, double yAngle, double zAngle);
        void scale(double scale);

        //TODO: add translation

        std::vector<Face> getFaces() const
        { return m_faces; }

        cv::Mat getTexture() const
        { return m_texture; }

    private:
        std::vector<Face> m_faces{};
        cv::Mat m_texture{};
    };
}

#endif //AR_OBJECT_HPP

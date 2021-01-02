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
        cv::Vec3d normal;
    };

    class Object
    {
    public:
        void addFace(std::vector<cv::Point3d> points, const cv::Vec3d& normal = cv::Vec3d(0,0,0));
        void rotate(double xAngle, double yAngle, double zAngle);
        void scale(double scale);
        void position(double xTranslation, double yTranslation, double zTranslation);

        //TODO: add translation

        std::vector<Face> getFaces() const
        { return m_faces; }

        cv::Vec3d getPosition() const
        {
            return m_position;
        }

    private:
        std::vector<Face> m_faces{};
        cv::Vec3d m_position = cv::Vec3d(0, 0, 0);
        int m_scale{1};
    };
}

#endif //AR_OBJECT_HPP

//
// Created by Simon on 22/12/2020.
//

#ifndef AR_OBJECT_HPP
#define AR_OBJECT_HPP

#include <vector>
#include <opencv2/core.hpp>

namespace arfs
{
    struct ObjectPoint
    {
        cv::Point3d coordinate;
        cv::Point2d textureCoordinate{};
    };

    struct Face
    {
        std::vector<ObjectPoint> points;
        cv::Vec3d normal;
    };

    class Object
    {
    public:
        void addFace(std::vector<ObjectPoint> points, const cv::Vec3d& normal = cv::Vec3d(0,0,0));
        void rotate(double xAngle, double yAngle, double zAngle);
        void scale(double scale);

        //TODO: add translation

        std::vector<Face> getFaces() const
        { return m_faces; }

    private:
        std::vector<Face> m_faces{};
    };
}

#endif //AR_OBJECT_HPP

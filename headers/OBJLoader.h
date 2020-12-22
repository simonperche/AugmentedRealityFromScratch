//
// Created by Simon on 21/12/2020.
//

#ifndef AR_OBJLOADER_H
#define AR_OBJLOADER_H

#include <string>
#include <opencv2/core/types.hpp>

namespace arfs
{
    struct Face
    {
        std::vector<cv::Point3d> points;
        cv::Vec3d normal;
    };

    class OBJLoader
    {
    public:
        OBJLoader(const std::string& filename);

        std::vector<Face> getFaces() const
        { return m_faces; }

        void rotate(double xAngle, double yAngle, double zAngle);

    private:
        std::vector<Face> m_faces;
    };
}

#endif //AR_OBJLOADER_H

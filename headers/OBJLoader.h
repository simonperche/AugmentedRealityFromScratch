//
// Created by Simon on 21/12/2020.
//

#ifndef AR_OBJLOADER_H
#define AR_OBJLOADER_H

#include <string>
#include <opencv2/core/types.hpp>

namespace arfs
{
    class OBJLoader
    {
    public:
        OBJLoader(const std::string& filename);

        std::vector<std::vector<cv::Point3d>> getFaces() const
        { return m_faces; }

        void rotate(double xAngle, double yAngle, double zAngle);

    private:
        std::vector<std::vector<cv::Point3d>> m_faces;
    };
}

#endif //AR_OBJLOADER_H

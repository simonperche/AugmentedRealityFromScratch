//
// Created by Simon on 22/12/2020.
//

#include <utility>

#include "../headers/Object.hpp"

namespace arfs
{
    Object::Object() : m_faces()
    {

    }

    void Object::scale(double scale)
    {
        for(auto& face : m_faces)
        {
            for(auto& point : face.points)
            {
                point *= scale;
            }
        }
    }

    void Object::rotate(double xAngle, double yAngle, double zAngle)
    {
        auto rot_x = cv::Matx33d(1, 0, 0,
                                 0, std::cos(xAngle), -std::sin(xAngle),
                                 0, std::sin(xAngle), std::cos(xAngle));
        auto rot_y = cv::Matx33d(std::cos(yAngle), 0, std::sin(yAngle),
                                 0, 1, 0,
                                 -std::sin(yAngle), 0, std::cos(yAngle));
        auto rot_z = cv::Matx33d(std::cos(zAngle), -std::sin(zAngle), 0,
                                 std::sin(zAngle), std::cos(zAngle), 0,
                                 0, 0, 1);
        auto rot = rot_x * rot_y * rot_z;
        for(auto& face : m_faces)
        {
            for(auto& point : face.points)
            {
                point = rot * point;
            }
        }
    }

    void Object::addFace(std::vector<cv::Point3d> points, const cv::Vec3d& normal)
    {
        m_faces.emplace_back(Face{std::move(points), normal});
    }
}


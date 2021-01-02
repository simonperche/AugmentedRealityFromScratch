//
// Created by Simon on 22/12/2020.
//

#include <utility>
#include <numeric>

#include "../headers/Utils.hpp"
#include "../headers/Object.hpp"

namespace arfs
{
    void Object::scale(double scale)
    {
        for(auto& face : m_faces)
        {
            for(auto& point : face.points)
            {
                point /= m_scale;
                point *= scale;
            }
        }

        m_scale = scale;
    }

    void Object::position(double xTranslation, double yTranslation, double zTranslation)
    {
        m_position = cv::Vec3d(xTranslation, yTranslation, zTranslation);
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

    void Object::addFace(const std::vector<cv::Point3d>& points, const std::vector<cv::Point2i>& textureCoordinate, const cv::Vec3d& normal)
    {
        cv::Scalar color;
        if(m_texture.empty())
        {
            // 180 because it looks better
            color = cv::Scalar(180,180,180);
        }
        else
        {
            auto centerTextureCoordinate = std::accumulate(textureCoordinate.begin(), textureCoordinate.end(), cv::Point2i{}) / double(textureCoordinate.size());
            std::vector<cv::Mat> channels(3);
            cv::split(m_texture, channels);
            auto b = channels[0].at<unsigned char>(centerTextureCoordinate);
            auto g = channels[1].at<unsigned char>(centerTextureCoordinate);
            auto r = channels[2].at<unsigned char>(centerTextureCoordinate);
            color = cv::Scalar(b,g,r);
        }

        m_faces.emplace_back(Face{points, color, normal});
    }

    void Object::setTextureImage(const std::string& filename)
    {
        m_texture = arfs::Utils::Image::loadImage(filename);
    }
}


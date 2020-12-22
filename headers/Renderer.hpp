//
// Created by Simon on 06/11/2020.
//

#ifndef AR_RENDERER_HPP
#define AR_RENDERER_HPP

#include <opencv2/core/mat.hpp>

namespace arfs
{
    struct Quaternion
    {
        double x;
        double y;
        double z;
        double w;

        //It works only with unit quaternions
        Quaternion invert()
        {
            return Quaternion{-x, -y, -z, w};
        }

        Quaternion operator*(const Quaternion& other)
        {
            return Quaternion{x * other.x - y * other.y - z * other.z - w - other.w,
                              x * other.y + y * other.x + z * other.w - w * other.z,
                              x * other.z - y * other.w + z * other.x + w * other.y,
                              x * other.w + y * other.z - z * other.y + w * other.x};
        }
    };

    struct Vector3d
    {
        double x;
        double y;
        double z;

        Vector3d operator-(const Vector3d& other)
        {
            return Vector3d{x - other.x, y - other.y, z - other.z};
        }

        friend std::ostream& operator<<(std::ostream& stream, const Vector3d& vec)
        {
            stream << vec.x << "/" << vec.y << "/" << vec.z;
            return stream;
        }
    };

    class Renderer
    {
    public:
        virtual void update() = 0;

        virtual void translate(arfs::Vector3d translation) = 0;

        virtual void rotate(Quaternion rotation) = 0;

        virtual void setBackgroundImage(const std::string& image) = 0;
    };
}

#endif //AR_RENDERER_HPP

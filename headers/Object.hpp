//
// Created by Simon on 22/12/2020.
//

#ifndef AR_OBJECT_HPP
#define AR_OBJECT_HPP

#include <vector>
#include <opencv2/core.hpp>

namespace arfs
{
    /**
     * @brief Represents an object face with points, color and normal
     */
    struct Face
    {
        std::vector<cv::Point3d> points;
        cv::Scalar color;
        cv::Vec3d normal;
    };

    /**
     * @brief Object in the 3D scene. It has several faces, a texture, a position and a scale.
     */
    class Object
    {
    public:
        /**
         * @brief Add a face to the object. The color is taken from the centroid of the face in texture image.
         * @param points coordinate of face points
         * @param textureCoordinate coordination (in percentage) in the texture image for each point
         * @param normal normal of the face
         */
        void addFace(const std::vector<cv::Point3d>& points, const std::vector<cv::Point2i>& textureCoordinate,
                     const cv::Vec3d& normal = cv::Vec3d(0, 0, 0));

        /**
         * @brief Set the texture image for the object
         * @param filename saved texture
         */
        void setTextureImage(const std::string& filename);

        /**
         * @brief Rotate the object. All angles are in radian.
         * @param xAngle
         * @param yAngle
         * @param zAngle
         */
        void rotate(double xAngle, double yAngle, double zAngle);

        /**
         * @brief Scale the object. Rescale an object apply scaling on original coordinate.
         * @param scale scale factor
         */
        void scale(double scale);

        /**
         * @brief Set the position of the object
         * @param x
         * @param y
         * @param z
         */
        void position(double x, double y, double z);

        std::vector<Face> getFaces() const
        { return m_faces; }

        cv::Mat getTexture() const
        { return m_texture; }

        cv::Vec3d getPosition() const
        { return m_position; }

    private:
        std::vector<Face> m_faces{};
        cv::Mat m_texture{};
        cv::Vec3d m_position = cv::Vec3d(0, 0, 0);
        double m_scale{1.};
    };
}

#endif //AR_OBJECT_HPP

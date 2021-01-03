//
// Created by Simon on 22/12/2020.
//

#ifndef AR_SCENE_HPP
#define AR_SCENE_HPP

#include <vector>
#include "Object.hpp"
#include "Camera.hpp"

namespace arfs
{
    /**
     * @brief 3D scene containing camera and objects.
     */
    class Scene
    {
    public:
        /**
         * @brief Construct a scene using a camera.
         * @param camera
         */
        explicit Scene(const arfs::Camera& camera) : m_camera(camera)
        {};

        /**
         * @brief Add an object to the scene using OBJLoader
         * @param objFilename filepath of .obj
         * @param mtlFilename filepath of .mtl (optional)
         */
        void addObject(const std::string& objFilename, const std::string& mtlFilename = {});

        /**
         * @brief Rotate all the scene. Angles are in radians.
         * @param xAngle
         * @param yAngle
         * @param zAngle
         */
        void rotate(double xAngle, double yAngle, double zAngle);

        /**
         * @brief Rotate one object using its index. 0 is the first added, 1, 2, 3... Angles are in radians.
         * @param objectIndex
         * @param xAngle
         * @param yAngle
         * @param zAngle
         */
        void rotate(int objectIndex, double xAngle, double yAngle, double zAngle);

        /**
         * @brief Scale all the scene using a scale factor.
         * @param scale scale factor
         */
        void scale(double scale);

        /**
         * @brief Scale one object using its index. 0 is the first added, 1, 2, 3...
         * @param objectIndex
         * @param scale scale factor
         */
        void scale(int objectIndex, double scale);

        /**
         * @brief Set the position of one object using its index. 0 is the first added, 1, 2, 3...
         * @param objectIndex
         * @param xPosition
         * @param yPosition
         * @param zPosition
         */
        void position(int objectIndex, double xPosition, double yPosition, double zPosition);

        arfs::Camera& getCamera()
        { return m_camera; }

        arfs::Camera getCamera() const
        { return m_camera; }

        std::vector<arfs::Object> getObjects() const
        { return m_objects; }

    private:
        std::vector<arfs::Object> m_objects{};
        arfs::Camera m_camera;
    };
}

#endif //AR_SCENE_HPP

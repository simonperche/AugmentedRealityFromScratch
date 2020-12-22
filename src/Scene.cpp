//
// Created by Simon on 22/12/2020.
//

#include "../headers/Scene.hpp"
#include "../headers/OBJLoader.hpp"

namespace arfs
{
    void Scene::addObject(const std::string& filename)
    {
        m_objects.emplace_back(arfs::OBJLoader::load(filename));
    }

    void Scene::rotate(double xAngle, double yAngle, double zAngle)
    {
        for(auto& object : m_objects)
            object.rotate(xAngle, yAngle, zAngle);
    }
}
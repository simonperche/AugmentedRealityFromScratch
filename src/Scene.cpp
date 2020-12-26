//
// Created by Simon on 22/12/2020.
//

#include "../headers/Scene.hpp"
#include "../headers/OBJLoader.hpp"

namespace arfs
{
    void Scene::addObject(const std::string& objFilename, const std::string& mtlFilename)
    {
        m_objects.emplace_back(arfs::OBJLoader::load(objFilename, mtlFilename));
    }

    void Scene::scale(double scale){
        for(auto& object : m_objects)
            object.scale(scale);
    }

    void Scene::rotate(double xAngle, double yAngle, double zAngle)
    {
        for(auto& object : m_objects)
            object.rotate(xAngle, yAngle, zAngle);
    }
}
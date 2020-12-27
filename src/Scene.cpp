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

    void Scene::scale(double scale){
        for(auto& object : m_objects)
            object.scale(scale);
    }

    void Scene::rotate(double xAngle, double yAngle, double zAngle)
    {
        for(auto& object : m_objects)
            object.rotate(xAngle, yAngle, zAngle);

    }

    void Scene::position(int objectIndex, double xPosition, double yPosition, double zPosition)
    {
        if(objectIndex >= m_objects.size())
            throw std::exception("Error : object index out of bound of the object list");

        m_objects.at(objectIndex).position(xPosition, yPosition, zPosition);
    }
}
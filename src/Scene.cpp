//
// Created by Simon on 22/12/2020.
//

#include <fstream>
#include <string>

#include "../headers/Scene.hpp"
#include "../headers/OBJLoader.hpp"
#include "../headers/Utils.hpp"
#include "../headers/exceptions.hpp"

namespace arfs
{
    void Scene::loadFromFile(const std::string& filename)
    {
        std::ifstream file(filename);
        std::string line;

        auto idSlash = filename.find_last_of('/');
        if(idSlash == std::string::npos)
            idSlash = filename.size()-2;

        auto path = filename.substr(0, idSlash+1);

        while(std::getline(file, line))
        {
            if(line.empty() || line[0] == '#') continue;

            auto split = arfs::Utils::split(line, ',');

            if(split.size() != 10) continue;

            addObject(path + split[0], path + split[1]);

            int objIndex{int(m_objects.size())-1};

            // Scale
            double scaleValue{1};
            try
            {
                if(!split[2].empty())
                    scaleValue = std::stod(split[2]);
            }
            catch(std::exception& e)
            {
                throw arfs::exceptions::IllFormedFile();
            }

            scale(objIndex, scaleValue);

            //Position
            double x{0},y{0},z{0};
            try
            {
                if(!split[3].empty())
                    x = std::stod(split[3]);
                if(!split[4].empty())
                    y = std::stod(split[4]);
                if(!split[5].empty())
                    z = std::stod(split[5]);
            }
            catch(std::exception&)
            {
                throw arfs::exceptions::IllFormedFile();
            }

            position(objIndex, x, y, z);

            arfs::AngleType angleType{arfs::AngleType::RAD};
            if(!split[6].empty() && split[6] == "DEG")
                angleType = arfs::AngleType::DEG;

            //Rotation
            double xRot{0},yRot{0},zRot{0};
            try
            {
                if(!split[7].empty())
                    xRot = std::stod(split[7]);
                if(!split[8].empty())
                    yRot = std::stod(split[8]);
                if(!split[9].empty())
                    zRot = std::stod(split[9]);
            }
            catch(std::exception& e)
            {
                throw arfs::exceptions::IllFormedFile();
            }

            if(angleType == arfs::AngleType::RAD)
                rotate(objIndex, xRot, yRot, zRot);
            else
                rotate(objIndex, arfs::Utils::Geometry::degToRad(xRot), arfs::Utils::Geometry::degToRad(yRot), arfs::Utils::Geometry::degToRad(zRot));
        }
    }

    void Scene::addObject(const std::string& objFilename, const std::string& mtlFilename)
    {
        m_objects.emplace_back(arfs::OBJLoader::load(objFilename, mtlFilename));
    }

    void Scene::scale(double scale)
    {
        for(auto& object : m_objects)
            object.scale(scale);
    }

    void Scene::rotate(double xAngle, double yAngle, double zAngle)
    {
        for(auto& object : m_objects)
            object.rotate(xAngle, yAngle, zAngle);
    }

    void Scene::scale(int objectIndex, double scale)
    {
        if(objectIndex >= m_objects.size())
            throw std::exception("Error : object index out of bound of the object list");

        m_objects[objectIndex].scale(scale);
    }

    void Scene::rotate(int objectIndex, double xAngle, double yAngle, double zAngle)
    {
        if(objectIndex >= m_objects.size())
            throw std::exception("Error : object index out of bound of the object list");

        m_objects[objectIndex].rotate(xAngle, yAngle, zAngle);

    }

    void Scene::position(int objectIndex, double xPosition, double yPosition, double zPosition)
    {
        if(objectIndex >= m_objects.size())
            throw std::exception("Error : object index out of bound of the object list");

        m_objects[objectIndex].position(xPosition, yPosition, zPosition);
    }
}
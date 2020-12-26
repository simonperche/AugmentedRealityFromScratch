//
// Created by Simon on 21/12/2020.
//

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "../headers/OBJLoader.hpp"
#include "../headers/Utils.hpp"

namespace arfs
{
    arfs::Object OBJLoader::load(const std::string& objFilename, const std::string& mtlFilename)
    {
        std::ifstream file(objFilename);
        std::string line;
        std::vector<cv::Point3d> vertices;
        std::vector<cv::Point2i> textures;
        std::vector<cv::Vec3d> normals;

        arfs::Object object{};

        // .mtl
        if(!mtlFilename.empty())
        {
            std::ifstream fileMTL(mtlFilename);
            while(std::getline(fileMTL, line))
            {
                auto split = arfs::Utils::split(line, ' ');

                if(split.size() != 2) continue;

                if(split[0] == "map_Kd")
                {
                    auto found = mtlFilename.find_last_of('/');
                    auto name = mtlFilename.substr(0, found) + '/' + split[1];
                    object.setTextureImage(name);
                    break;
                }
            }
        }

        // .obj
        while(std::getline(file, line))
        {
            if(line.empty()) continue;

            //TODO: change by Utils::split
            std::vector<std::string> line_split;
            std::istringstream iss(line);
            for(std::string s; iss >> s;)
                line_split.push_back(s);

            if(line_split.empty()) continue;

            //TODO: change catch by throwing an ill-formed exception

            if(line_split[0] == "v" || line_split[0] == "vn") // Vertices or normals
            {
                try
                {
                    double x = std::stod(line_split[1]);
                    double y = std::stod(line_split[2]);
                    double z = std::stod(line_split[3]);
                    if(line_split[0] == "v") // Vertices
                        vertices.emplace_back(cv::Point3d(x, y, z));
                    else // Normals
                        normals.emplace_back(cv::Vec3d(x, y, z));
                }
                catch(const std::exception& e)
                {
                    std::cout << e.what() << std::endl;
                    continue;
                }
            }
            else if(line_split[0] == "vt" && !object.getTexture().empty()) // Texture coordinates
            {
                if(object.getTexture().empty()) continue;

                try
                {
                    // Do not handle 1D or 3D texture
                    int x = int(std::stod(line_split[1]) * object.getTexture().cols);
                    int y = int((1-std::stod(line_split[2])) * object.getTexture().rows);
                    textures.emplace_back(cv::Point2i(x,y));
                }
                catch(const std::exception& e)
                {
                    std::cout << e.what() << std::endl;
                    continue;
                }
            }
            else if(line_split[0] == "f") // Faces
            {
                std::vector<cv::Point3d> points{};
                std::vector<cv::Point2i> textureCoordinate{};
                // Need the mean of normals to compute face normal
                auto sumNormals = cv::Vec3d(0,0,0);

                for(auto it = line_split.begin() + 1; it != line_split.end(); it++)
                {
                    auto faces_split = arfs::Utils::split(*it, '/');

                    if(faces_split.size() != 3) continue;

                    try
                    {
                        int id = std::stoi(faces_split[0]) - 1;
                        int idNormal = std::stoi(faces_split[2]) - 1;
                        points.emplace_back(vertices[id]);
                        sumNormals += normals[idNormal];

                        if(!faces_split[1].empty() && !textures.empty())
                        {
                            int idTexture = std::stoi(faces_split[1]) - 1;
                            textureCoordinate.emplace_back(textures[idTexture]);
                        }
                    }
                    catch(const std::exception& e)
                    {
                        std::cout << e.what() << std::endl;
                        continue;
                    }
                }

                object.addFace(points, textureCoordinate, sumNormals / (double)points.size());
            }
        }

        return object;
    }
}
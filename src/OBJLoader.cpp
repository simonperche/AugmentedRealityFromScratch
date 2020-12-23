//
// Created by Simon on 21/12/2020.
//

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <opencv2/core.hpp>
#include "../headers/OBJLoader.hpp"
#include "../headers/Utils.hpp"

namespace arfs
{
    arfs::Object OBJLoader::load(const std::string& filename)
    {
        std::ifstream file(filename);
        std::string line;
        std::vector<cv::Point3d> vertices;
        std::vector<cv::Vec3d> normals;

        arfs::Object object{};

        while(std::getline(file, line))
        {
            if(line.empty()) continue;

            std::vector<std::string> line_split;
            std::istringstream iss(line);
            for(std::string s; iss >> s;)
                line_split.push_back(s);

            if(line_split.empty()) continue;

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
            else if(line_split[0] == "f") // Faces
            {
                std::vector<cv::Point3d> face{};
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
                        face.emplace_back(vertices[id]);
                        sumNormals += normals[idNormal];
                    }
                    catch(const std::exception& e)
                    {
                        std::cout << e.what() << std::endl;
                        continue;
                    }
                }

                object.addFace(face, sumNormals/(double)face.size());
            }
        }

        return object;
    }
}
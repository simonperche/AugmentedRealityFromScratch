//
// Created by Simon on 21/12/2020.
//

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include "../headers/OBJLoader.h"

namespace arfs
{
    OBJLoader::OBJLoader(const std::string& filename)
    {
        std::ifstream file(filename);
        std::string line;
        std::vector<cv::Point3d> vertices;

        while(std::getline(file, line))
        {
            if(line.empty()) continue;

            std::vector<std::string> line_split;
            std::istringstream iss(line);
            for(std::string s; iss >> s;)
                line_split.push_back(s);

            if(line_split.empty()) continue;

            if(line_split[0] == "v") //Vertices
            {
                try
                {
                    double x = std::stod(line_split[1]);
                    double y = std::stod(line_split[2]);
                    double z = std::stod(line_split[3]);
                    vertices.emplace_back(cv::Point3d(x, y, z));
                }
                catch(const std::exception& e)
                {
                    e.what();
                    continue;
                }
            }
            else if(line_split[0] == "f") //Faces
            {
                std::vector<cv::Point3d> face{};
                for(auto it = line_split.begin() + 1; it != line_split.end(); it++)
                {
                    std::vector<std::string> faces_split{};
                    size_t pos{0};
                    std::string token{};
                    while((pos = it->find('/')) != std::string::npos)
                    {
                        token = it->substr(0, pos);
                        faces_split.emplace_back(token);
                        it->erase(0, pos + 1);
                    }

                    if(faces_split.size() != 3) continue;

                    try
                    {
                        int id = std::stoi(faces_split[0]) - 1;
                        face.emplace_back(vertices[id]);
                    }
                    catch(const std::exception& e)
                    {
                        e.what();
                        continue;
                    }
                }
                m_faces.emplace_back(face);
            }
        }
    }
}
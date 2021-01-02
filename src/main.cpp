//
// Created by Simon on 06/11/2020.
//

#include "../headers/Video.hpp"
#include "../headers/Scene.hpp"
#include "../headers/Renderer.hpp"
#include "../headers/TagDetection.hpp"
#include "../headers/ARTag.hpp"
#include "../headers/Utils.hpp"
#include "../headers/Camera.hpp"
#include "../headers/exceptions.hpp"
#include "../headers/CommandLineParser.hpp"

#include <iostream>

#include <ctime>

int main(int argc, char *argv[])
{
    //TODO: documentation (doxygen?)
    //TODO: Add config file for the scene

    auto cmdParser = arfs::CommandLineParser(argc, argv);

    int camId{-1};
    std::string videoFile{};

    // Arguments parsing
    try
    {
        cmdParser.getArgValue<int>("-w", "--webcam", camId);
        cmdParser.getArgValue<std::string>("--video", "-v", videoFile);

        if(cmdParser.getFlagValue("--help", "-h"))
        {
            std::cout << "Augmented Reality from scratch. Command line arguments :" << std::endl;
            std::cout << "\t[--webcam | -w] id : use webcam as video input. The first webcam is id 0." << std::endl
                      << "\t[--video | -v] filename : use video file as input. Filename is required." << std::endl
                      << "\t[--help | -h] : display this help and exit" << std::endl;
            return 0;
        }
    }
    catch(arfs::exceptions::BadCommandLineFormatting& e)
    {
        std::cout << "Error : " << e.what() << std::endl;
        return -1;
    }

    arfs::Video video;
    if(camId != -1)
        video = arfs::Video(camId, 1, 1);
    else if(!videoFile.empty())
        video = arfs::Video(videoFile, 0.5, 0.5);
    else
    {
        std::cout << "No video given. You should use -w or -v to specify an input video." << std::endl;
        return -1;
    }

    arfs::Camera camera{};
//    camera.calibrateAndSave("../resources/oneplus.cam",
//                            std::array<int,2>{11,11},
//                            "../resources/images_calibration/", 0.2);
    camera.loadParameters("../resources/webcam.cam");

    auto tagDetection = arfs::TagDetection(arfs::ARTag("../resources/marker.jpeg"));
    auto scene = arfs::Scene(camera);

    scene.addObject("../resources/monkey.obj", "../resources/monkey.mtl");
    scene.addObject("../resources/monkey.obj");
    scene.addObject("../resources/cube.obj");
    scene.addObject("../resources/low_poly_fox.obj", "../resources/low_poly_fox.mtl");

    scene.rotate(arfs::Utils::degToRad(90), arfs::Utils::degToRad(0), arfs::Utils::degToRad(180));
    scene.position(0, 60, -50, -250);
    scene.position(1, 0, 0, 0);
    scene.position(2, -40, -40, 20);
    scene.position(3, -250, 0, 0);
    scene.scale(50);
    scene.scale(3, 3);

    time_t tstart, tend;
    tstart = time(0);

    int workingFrames = 0;
    int notWorkingFrames = 0;
    cv::Mat frame;
    for(;;)
    {
        frame = video.getNextFrame();

        if(frame.empty() || arfs::Video::escIsPressed())
            break;

        auto renderFrame = frame.clone();

        auto tagDetected = tagDetection.update(frame);

        if(!tagDetected.empty())
        {
            scene.getCamera().updateProjectionMatrix(tagDetected);

            try
            {
                arfs::Renderer::render(renderFrame, scene);
            }
            catch(arfs::exceptions::EmptyProjectionMatrix& e)
            {
                std::cout << "Error : " << e.what() << std::endl;
                std::cout << "Maybe you forgot to update projection matrix of camera. "
                             "Or if tag was not detected, camera could not update projection matrix." << std::endl;
            }

            arfs::Renderer::drawPolygon(frame, tagDetected);
            workingFrames++;
        }
        else
        {
            notWorkingFrames++;
        }

        arfs::Utils::showImage(frame, "original");
        arfs::Utils::showImage(renderFrame, "render");
    }

    tend = time(0);
    std::cout << "It took " << difftime(tend, tstart) << " second(s)." << std::endl;
    std::cout << "Worked : " << workingFrames << ", didn't worked : " << notWorkingFrames << std::endl;

    return 0;
}
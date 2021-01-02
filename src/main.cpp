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
    std::string cameraParametersFile{};
    std::string tagFilename{};

    bool calibrateCamera;
    std::array<int, 2> checkerboardSize{0,0};
    std::string imgFolderCalibration{};
    bool needToTakePicturesForCalibration;

    // Arguments parsing
    try
    {
        cmdParser.getArgValue<int>("--webcam", "-w", camId);
        cmdParser.getArgValue<std::string>("--video", "-v", videoFile);
        cmdParser.getArgValue<std::string>("--camera-parameters", "-p", cameraParametersFile);
        cmdParser.getArgValue<std::string>("--tag", "-t", tagFilename);

        calibrateCamera = cmdParser.getFlagValue("--calibrate", "-c");
        cmdParser.getArgValue<int>("--checker-size-width", "", checkerboardSize[0]);
        cmdParser.getArgValue<int>("--checker-size-height", "", checkerboardSize[1]);
        cmdParser.getArgValue<std::string>("--images-folder", "", imgFolderCalibration);
        needToTakePicturesForCalibration = cmdParser.getFlagValue("--take-pictures", "");

        if(cmdParser.getFlagValue("--help", "-h"))
        {
            std::cout << "Augmented Reality from scratch. Command line arguments :" << std::endl;
            std::cout << "Video : " << std::endl;
            std::cout << "\t[--webcam | -w] id : use webcam as video input. The first webcam is id 0." << std::endl
                      << "\t[--video | -v] filename : use video file as input. Filename is required." << std::endl
                      << "\t[--tag | -t] filename : filename of tag to recognize. Must be an image." << std::endl
                      << "\t[--camera-parameters | -p] filename : use filename file from camera calibration." << std::endl;

            std::cout << "Calibration (optional) :" << std::endl;
            std::cout << "\t[--calibrate | -c] : calibrate the camera." << std::endl
                      << "\t--checker-size-width size : width of the checker board." << std::endl
                      << "\t--checker-size-height size : height of the checkerboard." << std::endl
                      << "\t--images-folder foldername : folder where to save calibration images." << std::endl
                      << "\t[--take-pictures] : if specified, the program will ask you to take checkerboard pictures, "
                         "otherwise load images from images folder." << std::endl;

            std::cout << "Other :" << std::endl;
            std::cout << "\t[--help | -h] : display this help and exit" << std::endl;
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

    if(cameraParametersFile.empty())
    {
        std::cout << "You should specify camera parameters with --camera-parameters or -p, "
                     "or calibrate your camera with --calibrate or -c.";
        return -1;
    }

    if(calibrateCamera)
    {
        if(checkerboardSize[0] == 0 || checkerboardSize[1] == 0 || imgFolderCalibration.empty())
        {
            std::cout << "You need to set the size of the checkerboard and the images folder in order to use calibration." << std::endl;
            return -1;
        }

        camera.calibrateAndSave(cameraParametersFile, checkerboardSize, imgFolderCalibration,
                                1, video, needToTakePicturesForCalibration);
    }

    camera.loadParameters(cameraParametersFile);

    if(tagFilename.empty())
    {
        std::cout << "You should specify a tag filename with --tag or -t." << std::endl;
        return -1;
    }
    auto tagDetection = arfs::TagDetection(arfs::ARTag(tagFilename));

    auto scene = arfs::Scene(camera);

    scene.addObject("../resources/monkey.obj", "../resources/monkey.mtl");
    scene.addObject("../resources/low_poly_fox.obj", "../resources/low_poly_fox.mtl");

    scene.rotate(arfs::Utils::Geometry::degToRad(90),
                 arfs::Utils::Geometry::degToRad(0),
                 arfs::Utils::Geometry::degToRad(180));
    scene.position(0, 100, 0, 0);
    scene.position(1, -100, 0, 0);
    scene.scale(60);
    scene.scale(1, 2.5);

    time_t tstart, tend;
    tstart = time(0);

    int workingFrames = 0;
    int notWorkingFrames = 0;
    cv::Mat frame;

    std::cout << "Press escape to finish." << std::endl;

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

        arfs::Utils::Image::showImage(frame, "original");
        arfs::Utils::Image::showImage(renderFrame, "render");
    }

    tend = time(0);
    std::cout << "It took " << difftime(tend, tstart) << " second(s)." << std::endl;
    std::cout << "Worked : " << workingFrames << " frames ; didn't worked : " << notWorkingFrames << " frames" << std::endl;

    return 0;
}
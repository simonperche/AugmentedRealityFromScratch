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

#include <iostream>

int main()
{
    //TODO: documentation (doxygen?)
    //TODO: Add args to launch the program from command line

    auto video = arfs::Video(0, 1,1);
//    auto video = arfs::Video("../resources/marker.mp4", 0.5, 0.5);

    arfs::Camera camera{};
//    camera.calibrateAndSave("../resources/oneplus.cam",
//                            std::array<int,2>{11,11},
//                            "../resources/images_calibration/", 0.2);
    camera.loadParameters("../resources/webcam.cam");

    auto tagDetection = arfs::TagDetection(arfs::ARTag("../resources/marker.jpeg"));

    auto scene = arfs::Scene(camera);
    scene.addObject("../resources/monkey.obj");
    scene.rotate(arfs::Utils::degToRad(90),arfs::Utils::degToRad(0),arfs::Utils::degToRad(180));
    scene.scale(100);

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
        }

        arfs::Utils::showImage(frame, "original");
        arfs::Utils::showImage(renderFrame, "render");
    }

    return 0;
}
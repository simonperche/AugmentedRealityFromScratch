//
// Created by Simon on 06/11/2020.
//

#include "../headers/Video.hpp"
#include "../headers/Scene.hpp"
#include "../headers/Renderer.hpp"
#include "../headers/TagDetection.hpp"
#include "../headers/ARTag.hpp"
#include "../headers/Utils.hpp"

int main()
{
    auto video = arfs::Video(0, 1,1);
//    auto video = arfs::Video("../resources/marker.mp4", 0.5, 0.5);

    auto tagDetection = arfs::TagDetection(arfs::ARTag("../resources/marker.jpeg"));
    auto scene = arfs::Scene();
    scene.addObject("../resources/low_poly_fox.obj");
    scene.getCamera().calibrateAndSave("../resources/webcam.cam");
    scene.rotate(arfs::Utils::degToRad(90),arfs::Utils::degToRad(0),arfs::Utils::degToRad(180));

    for(;;)
    {
        const auto& frame = video.getNextFrame();

        if(frame.empty() || arfs::Video::escIsPressed())
            break;

        auto renderFrame = frame.clone();

        auto tagDetected = tagDetection.update(frame);

        if(!tagDetected.empty())
        {
            scene.getCamera().updateProjectionMatrix(tagDetected);
            arfs::Renderer::render(renderFrame, scene);

            arfs::Renderer::drawPolygon(frame, tagDetected);
        }

        arfs::Utils::showImage(frame, "original");
        arfs::Utils::showImage(renderFrame, "render");
    }

    return 0;
}
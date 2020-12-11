//
// Created by Simon on 06/11/2020.
//

#include "../headers/Tracking.hpp"
#include "../headers/Video.hpp"
#include "../headers/View.inl"
#ifdef WITH_IRRLICHT
#include "../headers/IrrlichtRenderer.hpp"
#endif
#ifdef WITH_OPENGL
#include "../headers/OpenGLRenderer.hpp"
#endif
#include "../headers/Utils.hpp"
#include "../headers/Segmentation.hpp"

int main()
{
    auto video = arfs::Video("../video/aruco.mp4", 0.5, 0.5);

    for(;;)
    {
        const auto& frame = video.getNextFrame();

        if(frame.empty() || arfs::Video::escIsPressed())
            break;

        arfs::Utils::showImage(frame, "original");
        arfs::Utils::showImage(arfs::Segmentation::extractTag(frame), "threshold");
    }

    return 0;
}
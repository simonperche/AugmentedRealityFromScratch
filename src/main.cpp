//
// Created by Simon on 06/11/2020.
//

#include "../headers/Tracking.hpp"
#include "../headers/Video.hpp"
#include "../headers/View.inl"
#include "../headers/IrrlichtRenderer.hpp"
#include "../headers/Utils.hpp"

int main()
{
    auto video = arfs::Video("../video/translation.mp4");
    auto trackers = arfs::Tracking(video.getCurrentFrame());
    auto view = arfs::View<arfs::IrrlichtRenderer>(video.getCurrentFrame().cols, video.getCurrentFrame().rows);

    for(;;)
    {
        video.showFrame("tracker");
        const auto& frame = video.getNextFrame();

        if(video.getCurrentFrame().empty() || arfs::Video::escIsPressed())
            break;

        arfs::Utils::saveImage(video.getCurrentFrame(), "../background_img.png");

        trackers.update(frame);
        trackers.showTrackedPoint(frame);

        auto movement = trackers.getAvgMovement();
        view.translate(movement[0], movement[1], 0);
        view.setBackgroundImage("../background_img.png");
        view.update();
    }

    return 0;
}
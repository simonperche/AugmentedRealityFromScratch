//
// Created by Simon on 06/11/2020.
//

#include "../headers/Tracking.hpp"
#include "../headers/Video.hpp"

int main()
{
    auto video = arfs::Video("../video/translation.mp4");
    auto trackers = arfs::Tracking(video.getCurrentFrame());

    for(;;)
    {
        video.showFrame("tracker");
        const auto& frame = video.getNextFrame();

        if(video.getCurrentFrame().empty() || arfs::Video::escIsPressed())
            break;

        trackers.update(frame);
        trackers.showTrackedPoint(frame);
    }

    return 0;
}
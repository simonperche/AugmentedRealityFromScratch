//
// Created by Simon on 06/11/2020.
//

#ifndef AR_VIDEO_HPP
#define AR_VIDEO_HPP

#include <opencv2/videoio.hpp>

namespace arfs
{
    class Video
    {
    public:
        explicit Video(const std::string& name, double xResize = 1, double yResize = 1);
        explicit Video(int camId, double xResize = 1, double yResize = 1);
        static bool escIsPressed();
        void restartFromBeginning();
        cv::Mat getNextFrame();
        cv::Mat getCurrentFrame();

    private:
        cv::VideoCapture m_video{};
        cv::Mat m_currentFrame{};
        cv::Vec2d m_resizeValues{};
    };
}

#endif //AR_VIDEO_HPP

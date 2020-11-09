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
        Video(){};
        Video(std::string name);
        Video(int camId);
        static bool escIsPressed();
        void showFrame(const std::string& winname);
        cv::Mat getNextFrame();
        cv::Mat getCurrentFrame()
        { return m_currentFrame; };

    private:
        cv::VideoCapture m_video{};
        cv::Mat m_currentFrame{};
    };
}

#endif //AR_VIDEO_HPP

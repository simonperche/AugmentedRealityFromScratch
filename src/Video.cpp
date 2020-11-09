//
// Created by Simon on 06/11/2020.
//

#include <opencv2/highgui.hpp>
#include "../headers/Video.hpp"

namespace arfs
{
    Video::Video(std::string name)
            : m_video(name)
    {
        if(!m_video.isOpened())
            throw std::exception("Error : the video is not opened. Please check the file name.");

        m_currentFrame = getNextFrame();
    }

    Video::Video(int camId)
            : m_video(camId)
    {
        if(!m_video.isOpened())
            throw std::exception("Error : the webcam is not opened. Please check the id.");
        m_currentFrame = getNextFrame();
    }

    cv::Mat Video::getNextFrame()
    {
        m_video >> m_currentFrame;
        return m_currentFrame;
    }

    bool Video::escIsPressed()
    {
        return cv::waitKey(1) == 27;
    }

    void Video::showFrame(const std::string& winname)
    {
        cv::imshow(winname, m_currentFrame);
    }

}


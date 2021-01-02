//
// Created by Simon on 06/11/2020.
//

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "../headers/Video.hpp"

namespace arfs
{
    Video::Video(const std::string& name, double xResize, double yResize)
            : m_video(name), m_resizeValues(xResize, yResize)
    {
        if(!m_video.isOpened())
            throw std::exception("Error : the video is not opened. Please check the file name.");

        m_video >> m_currentFrame;
    }

    Video::Video(int camId, double xResize, double yResize)
            : m_video(camId, cv::CAP_DSHOW), m_resizeValues(xResize, yResize)
    {
        if(!m_video.isOpened())
            throw std::exception("Error : the webcam is not opened. Please check the id.");
        m_video >> m_currentFrame;
    }

    cv::Mat Video::getNextFrame()
    {
        m_video >> m_currentFrame;
        return getCurrentFrame();
    }

    cv::Mat Video::getCurrentFrame()
    {
        cv::Mat resized;
        if(!m_currentFrame.empty())
            cv::resize(m_currentFrame, resized, cv::Size(), m_resizeValues[0], m_resizeValues[1]);
        return resized;
    }

    bool Video::escIsPressed()
    {
        return cv::waitKey(1) == 27;
    }

    void Video::restartFromBeginning()
    {
        m_video.set(cv::CAP_PROP_POS_FRAMES, 0);
    }
}


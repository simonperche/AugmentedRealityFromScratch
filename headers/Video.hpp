//
// Created by Simon on 06/11/2020.
//

#ifndef AR_VIDEO_HPP
#define AR_VIDEO_HPP

#include <opencv2/videoio.hpp>

namespace arfs
{
    /**
     * @brief Video wrapper. Handle video from file and webcam.
     */
    class Video
    {
    public:
        /**
         * @brief Construct an empty video.
         */
        Video()
        {};

        /**
         * @brief Construct a video from file.
         * @param name filepath of video
         * @param xResize resize factor (default 1)
         * @param yResize resize factor (default 1)
         */
        explicit Video(const std::string& name, double xResize = 1, double yResize = 1);

        /**
         * @brief Construct a video from webcam.
         * @param camId Id of camera, using OpenCV id (0 is the first, 1...)
         * @param xResize resize factor (default 1)
         * @param yResize resize factor (default 1)
         */
        explicit Video(int camId, double xResize = 1, double yResize = 1);

        /**
         * @brief Restart the video from the beginning. Do nothing if Video was built with webcam.
         */
        void restartFromBeginning();

        /**
         * @brief Increment the frame and return the next one.
         * @details It must be called in order to continue the video.
         * @return the next frame.
         */
        cv::Mat getNextFrame();

        /**
         * @brief Return the current frame.
         * @return the current frame
         */
        cv::Mat getCurrentFrame();

    private:
        cv::VideoCapture m_video{};
        cv::Mat m_currentFrame{};
        cv::Vec2d m_resizeValues{};
    };
}

#endif //AR_VIDEO_HPP

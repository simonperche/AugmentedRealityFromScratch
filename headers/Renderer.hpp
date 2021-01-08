//
// Created by Simon on 22/12/2020.
//

#ifndef AR_RENDERER_HPP
#define AR_RENDERER_HPP

#include <opencv2/core/mat.hpp>
#include "Scene.hpp"

/**
 * @brief Render functions
 */
namespace arfs::Renderer
{
    /**
     * @brief Render a scene on a frame.
     * @param frame
     * @param scene
     */
    void render(cv::Mat& frame, const arfs::Scene& scene);

    /**
     * @brief Draw a polygon on a frame.
     * @param frame
     * @param points array of polygon points
     */
    void drawPolygon(const cv::Mat& frame, const std::vector<cv::Point>& points);
}

#endif //AR_RENDERER_HPP

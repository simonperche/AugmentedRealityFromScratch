//
// Created by Simon on 22/12/2020.
//

#ifndef AR_RENDERER_HPP
#define AR_RENDERER_HPP

#include <opencv2/core/mat.hpp>
#include "Scene.hpp"

namespace arfs
{
    class Renderer
    {
    public:
        static void render(const cv::Mat& frame, const arfs::Scene& scene);
        static void drawPolygon(const cv::Mat& frame, const std::vector<cv::Point>& points);

    private:
        static void augmentObject(const cv::Mat& frame, const arfs::Object& object, const arfs::Camera& camera);
        static std::vector<cv::Point2i> projectPoint(const std::vector<arfs::ObjectPoint>& points, const cv::Mat& projectionMatrix);
    };
}

#endif //AR_RENDERER_HPP

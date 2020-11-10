//
// Created by Simon on 06/11/2020.
//

#ifndef AR_RENDERER_HPP
#define AR_RENDERER_HPP

#include <opencv2/core/mat.hpp>

namespace arfs
{
    class Renderer
    {
    public:
        virtual void update() = 0;
        virtual void translate(double x, double y, double z) = 0;
        virtual void setBackgroundImage(const std::string& image) = 0;
    };
}

#endif //AR_RENDERER_HPP

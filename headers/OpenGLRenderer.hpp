//
// Created by Simon on 19/11/2020.
//

#ifndef AR_OPENGLRENDERER_HPP
#define AR_OPENGLRENDERER_HPP

#include "Renderer.hpp"

namespace arfs
{
    class OpenGLRenderer : Renderer
    {
    public:
        OpenGLRenderer(int x, int y);
        void update();
        void translate(arfs::Vector3d translation);
        void rotate(Quaternion rotation);
        void setBackgroundImage(const std::string& image);
    private:

    };
}

#endif //AR_OPENGLRENDERER_HPP

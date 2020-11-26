//
// Created by Simon on 06/11/2020.
//

#ifndef AR_VIEW_INL
#define AR_VIEW_INL

#include <type_traits>
#include "Renderer.hpp"

namespace arfs
{
    template<class MyRenderer>
    class View
    {
        static_assert(std::is_base_of<Renderer, MyRenderer>::value, "The renderer must inherit from Renderer.");
    public:
        View(int x, int y) : m_renderer(x, y) {};
        void update()
        {
            m_renderer.update();
        }

        void translate(arfs::Vector3d v)
        {
            m_renderer.translate(v);
        };

        void rotate(arfs::Quaternion q)
        {
            m_renderer.rotate(q);
        }

        void setBackgroundImage(const std::string& filename)
        {
            m_renderer.setBackgroundImage(filename);
        };

    private:
        MyRenderer m_renderer{};
    };
}

#endif //AR_VIEW_INL

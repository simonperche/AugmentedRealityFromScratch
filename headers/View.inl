//
// Created by Simon on 06/11/2020.
//

#ifndef AR_VIEW_INL
#define AR_VIEW_INL

#include <type_traits>
#include "Renderer.hpp"

template<class MyRenderer>
class View
{
    static_assert(std::is_base_of<Renderer, MyRenderer>::value, "The renderer must inherit from Renderer.");
public:
    void update()
    {
        m_renderer.update();
    }
private:
    MyRenderer m_renderer{};
};

#endif //AR_VIEW_INL

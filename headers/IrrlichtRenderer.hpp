//
// Created by Simon on 06/11/2020.
//

#ifndef AR_IRRLICHT_RENDERER_HPP
#define AR_IRRLICHT_RENDERER_HPP

#include <irrlicht.h>
#include "Renderer.hpp"

namespace impl
{
    static void deviceDestructor(irr::IrrlichtDevice* ptr)
    {
        if(ptr)
            ptr->drop();
    }
}

namespace arfs
{
    class IrrlichtRenderer : Renderer
    {
    public:
        IrrlichtRenderer(int x, int y);
        void update() override;
        void translate(arfs::Vector3d translation) override;
        void rotate(arfs::Quaternion q) override;
        void setBackgroundImage(const std::string& image) override;

    private:
        std::unique_ptr<irr::IrrlichtDevice, decltype(&impl::deviceDestructor)> m_device;
        irr::video::IVideoDriver* m_driver{};
        irr::scene::ISceneManager* m_smgr{};
        irr::gui::IGUIEnvironment* m_guienv{};
        irr::scene::ICameraSceneNode* m_camera{};
        irr::scene::IMeshSceneNode* m_mesh{};
        irr::video::ITexture* m_backgroundImage{};
    };
}

#endif //AR_IRRLICHT_RENDERER_HPP

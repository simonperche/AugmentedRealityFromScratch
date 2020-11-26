//
// Created by Simon on 06/11/2020.
//

#include <iostream>
#include "../headers/IrrlichtRenderer.hpp"

namespace arfs
{
    IrrlichtRenderer::IrrlichtRenderer(int x, int y) :
            m_device(irr::createDevice(irr::video::EDT_SOFTWARE, irr::core::dimension2d<irr::u32>(x, y), 16, false,
                                       false, false, nullptr), &impl::deviceDestructor)
    {
        m_driver = m_device->getVideoDriver();
        m_smgr = m_device->getSceneManager();
        m_guienv = m_device->getGUIEnvironment();

        //irr::scene::IAnimatedMesh* mesh = m_smgr->getMesh("../object.obj");
        //m_smgr->addMeshSceneNode(mesh);
        m_mesh = m_smgr->addCubeSceneNode(2.0f, nullptr, -1, irr::core::vector3df(0,0,20));
        m_mesh->setMaterialTexture(0, m_driver->getTexture("../textures/wood.png"));
        m_smgr->addLightSceneNode(nullptr, irr::core::vector3df(-15,5,-105),
                                  irr::video::SColorf(1.0f, 1.0f, 1.0f));

        // set ambient light
        m_smgr->setAmbientLight(irr::video::SColor(0,60,60,60));

        m_camera = m_smgr->addCameraSceneNode(nullptr, irr::core::vector3df(0, 0, 0));
        m_camera->bindTargetAndRotation(true);
    }

    void IrrlichtRenderer::update()
    {
        m_driver->beginScene(true, true, irr::video::SColor(255, 100, 101, 140));
        if(m_backgroundImage)
            m_driver->draw2DImage(m_backgroundImage, irr::core::position2d<irr::s32>(0, 0));
        m_smgr->drawAll();
        m_guienv->drawAll();
        m_driver->endScene();
    }

    void IrrlichtRenderer::translate(arfs::Vector3d translation)
    {
        m_camera->setPosition(m_camera->getPosition() - irr::core::vector3d<irr::f32>(translation.x, translation.y, translation.z));
        std::cout << translation << std::endl;
        std::cout << m_camera->getPosition().X << "/" << m_camera->getPosition().Y << "/" << m_camera->getPosition().Z << std::endl;
    }

    void IrrlichtRenderer::rotate(arfs::Quaternion q)
    {
        auto rotation = irr::core::quaternion(q.x, q.y, q.z, q.w);
        irr::core::vector3d<irr::f32> angleDiff{};
        rotation.toEuler(angleDiff);

        auto pos = m_camera->getPosition();
        m_camera->setPosition(irr::core::vector3d<irr::f32>(0,0,0));
        m_camera->setRotation(m_camera->getRotation() + angleDiff);
        m_camera->setPosition(pos);
    }

    void IrrlichtRenderer::setBackgroundImage(const std::string& filename)
    {
        if(m_backgroundImage)
            m_driver->removeTexture(m_backgroundImage);
        //TODO: find a way to use std::string filename
        m_backgroundImage = m_driver->getTexture("background_img.png");
    }
}

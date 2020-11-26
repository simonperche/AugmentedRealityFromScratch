//
// Created by Simon on 06/11/2020.
//

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "../headers/Tracking.hpp"
#include "../headers/Video.hpp"
#include "../headers/View.inl"
#ifdef WITH_IRRLICHT
#include "../headers/IrrlichtRenderer.hpp"
#endif
#ifdef WITH_OPENGL
#include "../headers/OpenGLRenderer.hpp"
#endif
#include "../headers/Utils.hpp"
#include "../headers/Segmentation.hpp"
#include "../headers/IrrlichtRenderer.hpp"

int main()
{
    auto view = arfs::View<arfs::OpenGLRenderer>(500, 500);
//    auto video = arfs::Video("../video/movement.mp4", 0.5, 0.5);
////    std::cout << video.getCurrentFrame().cols << video.getCurrentFrame().rows << std::endl;
////    auto trackers = arfs::Tracking(video.getCurrentFrame());
////    auto view = arfs::View<arfs::IrrlichtRenderer>(video.getCurrentFrame().cols, video.getCurrentFrame().rows);
//
//    unsigned int i{0};
//    if(std::filesystem::is_empty(std::filesystem::path("../images/")))
//    {
//        for(;;)
//        {
//            video.showFrame("tracker");
//            const auto& frame = video.getNextFrame();
//
//            if(frame.empty() || arfs::Video::escIsPressed())
//                break;
//
//            const std::string& nb = std::to_string(i);
//            arfs::Utils::saveImage(frame, "../images/img" + std::string(3 - nb.length(), '0') + nb + ".jpg");
//
//            i++;
////
////        arfs::Utils::saveImage(video.getCurrentFrame(), "background_img.png");
////
////        trackers.update(frame);
////        trackers.showTrackedPoint(frame);
//
////        arfs::Segmentation::segmentation(frame, trackers.getDepthPoints(frame));
//
////        auto movement = trackers.getAvgMovement();
////        view.translate(movement[0], movement[1], 0);
////        view.setBackgroundImage("background_img.png");
////        view.update();
//        }
//    }
//
//    if(std::filesystem::is_empty(std::filesystem::path("../colmap_workspace/")))
//    {
//        std::system("..\\COLMAP\\COLMAP.bat automatic_reconstructor "
//                    "--workspace_path ..\\colmap_workspace "
//                    "--image_path ..\\images "
//                    "--use_gpu 0");
//    }
//
//    auto file_cam_pos = std::ifstream{"../colmap_workspace/sparse/0/images.txt"};
//    if(!file_cam_pos)
//    {
//        std::system("..\\COLMAP\\COLMAP.bat model_converter "
//                    "--input_path ..\\colmap_workspace\\sparse\\0 "
//                    "--output_path ..\\colmap_workspace\\sparse\\0 "
//                    "--output_type TXT");
//    }
//
//    const unsigned int numberOfFrames{i+1};
//    auto rotations = std::vector<arfs::Quaternion>(numberOfFrames);
//    auto translations = std::vector<arfs::Vector3d>(numberOfFrames);
//
//    bool isEven = true;
//    std::string line{};
//    while(std::getline(file_cam_pos, line))
//    {
//        if(line[0] == '#')
//            continue;
//
//        if(isEven)
//        {
//            std::vector<std::string> tokens{};
//            auto lineStringStream = std::stringstream(line);
//            std::string tmp{};
//            while(std::getline(lineStringStream, tmp, ' '))
//                tokens.emplace_back(tmp);
//
//            // The file is formed with IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME
//            //                         0        1  2  3  4  5  6  7  8         9
//            rotations.emplace_back(arfs::Quaternion{std::stod(tokens[2]), std::stod(tokens[3]),
//                                                    std::stod(tokens[4]), std::stod(tokens[1])});
//            translations.emplace_back(arfs::Vector3d{std::stod(tokens[5]), std::stod(tokens[6]),
//                                                     std::stod(tokens[7])});
//        }
//
//        isEven = !isEven;
//    }
//
//    video.restartFromBeginning();
//
////    auto trackers = arfs::Tracking(video.getCurrentFrame());
//#ifdef WITH_IRRLICHT
//    auto view = arfs::View<arfs::IrrlichtRenderer>(video.getCurrentFrame().cols, video.getCurrentFrame().rows);
//#endif
//
//    i = 0;
//    for(;;)
//    {
//        video.showFrame("tracker");
//        const auto& frame = video.getNextFrame();
//
//        if(frame.empty() || arfs::Video::escIsPressed())
//            break;
//
//        arfs::Utils::saveImage(frame, "background_img.png");
//
////        trackers.update(frame);
////        trackers.showTrackedPoint(frame);
////
////        arfs::Segmentation::segmentation(frame, trackers.getDepthPoints(frame));
//
////        auto movement = trackers.getAvgMovement();
//
//        view.setBackgroundImage("background_img.png");
//        view.update();
//        if(i != 0)
//        {
//            view.translate(translations[i] - translations[i-1]);
////            view.rotate(rotations[i] * rotations[i-1].invert());
//        }
//
//
//
//        i++;
//    }
//
//
//
////    cv::Mat img = cv::imread("../background_img.png");
////
////    arfs::Segmentation::segmentation(img, std::vector<std::pair<cv::Point2d, double>>());
//
//    return 0;
}
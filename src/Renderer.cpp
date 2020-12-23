//
// Created by Simon on 22/12/2020.
//

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "../headers/exceptions.hpp"
#include "../headers/Renderer.hpp"
#include "../headers/Utils.hpp"

namespace arfs
{
    void Renderer::render(const cv::Mat& frame, const arfs::Scene& scene)
    {
        if(scene.getCamera().getProjectionMatrix().empty())
            throw arfs::exceptions::EmptyProjectionMatrix();

        for(const auto& object : scene.getObjects())
        {
            augmentObject(frame, object, scene.getCamera());
        }
    }

    void Renderer::augmentObject(const cv::Mat& frame, const Object& object, const arfs::Camera& camera)
    {
        auto faces = object.getFaces();
        auto tagProjectionSize = camera.getTagProjectionSize();

        // Painter's algorithm
        //TODO: fix painter's algorithm when the camera is too close or too titled in relation to the tag
        cv::Mat translation = (camera.getIntrinsicParameters().inv() * camera.getProjectionMatrix()).col(3);
        std::sort(faces.begin(), faces.end(), [&](const Face& a, const Face& b)
        {
            auto a_mean = cv::Point3d(0, 0, 0);
            auto b_mean = cv::Point3d(0, 0, 0);
            for(const auto& point : a.points)
                a_mean += point;
            for(const auto& point : b.points)
                b_mean += point;
            a_mean /= (double) a.points.size();
            b_mean /= (double) b.points.size();
            return cv::norm(cv::Point3d(translation) - a_mean) < cv::norm(cv::Point3d(translation) - b_mean);
        });

        for(auto& face : faces)
        {
            for(auto& point : face.points)
            {
                //Center
                point.x += int(tagProjectionSize / 2);
                point.y += int(tagProjectionSize / 2);
            }
            auto scene_points = projectPoint(face.points, camera.getProjectionMatrix());

            auto light = cv::Vec3d(-1, 1, -1);
            auto angle = arfs::Utils::angleBetween(face.normal, light, arfs::AngleType::DEG);
            auto lightValue = (angle * 255) / 180;
            cv::fillConvexPoly(frame, scene_points, cv::Scalar(lightValue, lightValue, lightValue));
        }
    }

    std::vector<cv::Point2i> Renderer::projectPoint(const std::vector<cv::Point3d>& points, const cv::Mat& projectionMatrix)
    {
        std::vector<cv::Point2i> scene_points(points.size());
        for(int i = 0; i < points.size(); i++)
        {
            cv::Mat_<double> src(4, 1);

            src(0, 0) = points[i].x;
            src(1, 0) = points[i].y;
            src(2, 0) = points[i].z;
            src(3, 0) = 1;
            cv::Mat tmp = projectionMatrix * src;
            tmp /= tmp.at<double>(2, 0);
            scene_points[i].x = int(tmp.at<double>(0, 0));
            scene_points[i].y = int(tmp.at<double>(1, 0));
        }

        return scene_points;
    }

    void Renderer::drawPolygon(const cv::Mat& frame, const std::vector<cv::Point>& points)
    {
        auto size = points.size();
        for(size_t i = 0; i < size - 1; i++)
            cv::line(frame, points[i], points[i + 1], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::line(frame, points[size - 1], points[0], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
}
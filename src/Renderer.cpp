//
// Created by Simon on 22/12/2020.
//

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "../headers/exceptions.hpp"
#include "../headers/Renderer.hpp"
#include "../headers/Utils.hpp"

#include <iostream>

namespace arfs
{
    void Renderer::render(cv::Mat& frame, const arfs::Scene& scene)
    {
        if(scene.getCamera().getProjectionMatrix().empty())
            throw arfs::exceptions::EmptyProjectionMatrix();

        for(auto& object : scene.getObjects())
        {
            augmentObject(frame, object, scene.getCamera());
        }
    }

    double isOnLine(cv::Point2i p1, cv::Point2i p2, cv::Point2i p)
    {
        int x1 = p1.x;
        int y1 = p1.y;
        int x2 = p2.x;
        int y2 = p2.y;
        int x = p.x;
        int y = p.y;
        double AB = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
        double AP = sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
        double PB = sqrt((x2-x)*(x2-x)+(y2-y)*(y2-y));
        return AB - AP + PB;
        //if(AB == AP + PB)
        //    return true;
    }

    void Renderer::augmentObject(cv::Mat& frame, const Object& object, const arfs::Camera& camera)
    {
        auto faces = object.getFaces();
        auto tagProjectionSize = camera.getTagProjectionSize();

        // Painter's algorithm
        //TODO: fix painter's algorithm with multiples objects
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

        int i1 = 0;
        for(auto& face : faces)
        {

            for(auto& point : face.points)
            {
                //Center
                point.x += int(tagProjectionSize / 2);
                point.y += int(tagProjectionSize / 2);
            }
            int i2 = 0;

            std::vector<std::vector<cv::Vec3d>> normals = {};
            for(int i = 0; i<face.points.size(); i++)
            {
                std::vector<cv::Vec3d> norms = {face.normal}; //std::vector<cv::Vec3d>();
                normals.push_back(norms);
            }
            for(auto& sface : faces)
            {
                if(i1 != i2)
                {
                    for(auto& spoint : sface.points)
                    {
                        for(int j = 0; j<face.points.size(); j++)
                        {
                            if(face.points[j].x == spoint.x && face.points[j].y == spoint.y)
                            {
                                normals[j].push_back(sface.normal);
                            }
                        }
                    }
                }
                i2++;
            }
            i1++;

            std::vector<cv::Vec3d> avgNormals = {};
            for(std::vector<cv::Vec3d> ptNormal : normals)
            {
                cv::Vec3d avg = cv::Vec3d(0, 0, 0);
                for(cv::Vec3d pt : ptNormal)
                {
                    avg += pt;
                }
                avg = avg * (1.0/ptNormal.size());
                avgNormals.push_back(avg);
            }

            auto light = cv::Vec3d(-1, 1, -1);

            std::vector<double> intensities = {};
            for(cv::Vec3d avgNormal : avgNormals)
            {
                auto angle = arfs::Utils::angleBetween(avgNormal, light, arfs::AngleType::DEG);
                auto lightValue = (angle * 255) / 180;
                intensities.push_back(lightValue);
            }

            auto scene_points = projectPoint(face.points, camera.getProjectionMatrix());

            int minX = scene_points[0].x;
            int maxX = scene_points[0].x;
            int minY = scene_points[0].y;
            int maxY = scene_points[0].y;

            for (auto& point : scene_points)
            {
                if(point.x > maxX) maxX = point.x;
                if(point.x < minX) minX = point.x;
                if(point.y > maxY) maxY = point.y;
                if(point.y < minY) minY = point.y;
            }

            cv::Mat faceMat = cv::Mat::zeros(frame.size(), CV_8UC1);

            if(scene_points.size() == 3)
            {
                cv::line(faceMat, scene_points[0], scene_points[1], cv::Scalar(255));
                cv::line(faceMat, scene_points[1], scene_points[2], cv::Scalar(255));
                cv::line(faceMat, scene_points[2], scene_points[0], cv::Scalar(255));
                //cv::imshow("faceMat", faceMat);
            } else{
                cv::line(faceMat, scene_points[0], scene_points[1], cv::Scalar(255));
                cv::line(faceMat, scene_points[1], scene_points[2], cv::Scalar(255));
                cv::line(faceMat, scene_points[2], scene_points[3], cv::Scalar(255));
                cv::line(faceMat, scene_points[3], scene_points[0], cv::Scalar(255));
            }

            std::vector<std::vector<cv::Point> > contours;

            findContours( faceMat, contours, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_L1);

            /*
            cv::Mat output = cv::Mat::zeros(faceMat.size(), CV_8UC3);
            cv::RNG rng(12345);
            for( size_t i = 0; i< contours.size(); i++ ){
                cv::Scalar colorC = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                drawContours(output, contours, i, colorC, 1);
            }
            imshow("contours", output );
            //cv::waitKey(0);*/

            if(contours.size() > 0)
            {
                for(int y = minY; y < maxY; y++)
                {
                    int minXline = maxX;
                    int maxXline = minX;
                    for(int x = minX; x < maxX; x++)
                    {
                        if(cv::pointPolygonTest(contours[0], cv::Point2f((float)x, (float)y), true) > 0)
                        {
                            if(x < minXline) minXline = x;
                            if(x > maxXline) maxXline = x;
                        }
                    }

                    int ind1min = 0;
                    int ind2min = 0;
                    double minValueForMin = -1;
                    int ind1max = 0;
                    int ind2max = 0;
                    double minValueForMax = -1;
                    for(int i = 0; i<intensities.size(); i++)
                    {
                        for(int j = 0; j<intensities.size(); j++)
                        {
                            if(i != j)
                            {
                                double tempValue = isOnLine(scene_points[i], scene_points[j], cv::Point2i(minXline, y));
                                if(tempValue < minValueForMin || minValueForMin == -1)
                                {
                                    ind1min = i;
                                    ind2min = j;
                                    minValueForMin = tempValue;
                                }
                                if(tempValue > minValueForMax || minValueForMax == -1)
                                {
                                    ind1max = i;
                                    ind2max = j;
                                    minValueForMax = tempValue;
                                }
                            }
                        }
                    }

                    if(scene_points[ind1min].y > scene_points[ind2min].y)
                    {
                        int swap = ind1min;
                        ind1min = ind2min;
                        ind2min = swap;
                    }
                    if(scene_points[ind1max].y > scene_points[ind2max].y)
                    {
                        int swap = ind1max;
                        ind1max = ind2max;
                        ind2max = swap;
                    }

                    double ia, ib;
                    std::cout << ind1min << " " << ind2min << " --- " << ind1max << " " << ind2max << std::endl;
                    if(intensities.size() == 4)
                    {
                        /*ia = intensities[3] - (intensities[3] - intensities[0])*(scene_points[3].y - y)/(scene_points[3].y - scene_points[0].y);
                        ib = intensities[2] - (intensities[2] - intensities[1])*(scene_points[2].y - y)/(scene_points[2].y - scene_points[1].y);*/
                        ia = intensities[ind2min] - (intensities[ind2min] - intensities[ind1min])*(scene_points[ind2min].y - y)/(scene_points[ind2min].y - scene_points[ind1min].y);
                        ib = intensities[ind2max] - (intensities[ind2max] - intensities[ind2min])*(scene_points[ind2max].y - y)/(scene_points[ind2max].y - scene_points[ind2min].y);
                    }
                    else if(intensities.size() == 3)
                    {
                        /*ia = intensities[2] - (intensities[2] - intensities[0])*(scene_points[2].y - y)/(scene_points[2].y - scene_points[0].y);
                        ib = intensities[2] - (intensities[2] - intensities[1])*(scene_points[2].y - y)/(scene_points[2].y - scene_points[1].y);*/
                        ia = intensities[ind2min] - (intensities[ind2min] - intensities[ind1min])*(scene_points[ind2min].y - y)/(scene_points[ind2min].y - scene_points[ind1min].y);
                        ib = intensities[ind2max] - (intensities[ind2max] - intensities[ind2min])*(scene_points[ind2max].y - y)/(scene_points[ind2max].y - scene_points[ind2min].y);
                    }

                    for(int x = minX; x < maxX; x++)
                    {
                        try{
                            if(cv::pointPolygonTest(contours[0], cv::Point2f((float)x, (float)y), true) > 0)
                            {
                                /*double total = 0;
                                double sum = 0;
                                for(int i = 0; i<scene_points.size(); i++)
                                {
                                    int xp = scene_points[i].x;
                                    int yp = scene_points[i].y;
                                    double distance = sqrt((x-xp)*(x-xp) + (y-yp)*(y-yp));
                                    total += distance * intensities[i];
                                    sum += distance;
                                }
                                total /= sum;*/
                                double ip = ib - (ib - ia) * (maxXline - x) / (maxXline - minXline);
                                cv::Vec3b color = cv::Vec3b(ip, ip, ip);
                                frame.at<cv::Vec3b>(y, x) = color;
                            }

                        }catch(const std::exception& e)
                        {
                            std::cout << e.what() << std::endl;
                        }

                    }
                }
            }


            //TODO: add more shading methods
            //Flat shading
            /*auto light = cv::Vec3d(-1, 1, -1);
            auto angle = arfs::Utils::angleBetween(face.normal, light, arfs::AngleType::DEG);
            auto lightValue = (angle * 255) / 180;*/

            //cv::fillConvexPoly(frame, scene_points, cv::Scalar(lightValue, lightValue, lightValue));

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
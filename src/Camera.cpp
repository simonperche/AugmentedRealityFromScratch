//
// Created by Simon on 22/12/2020.
//

#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "../headers/Camera.hpp"
#include "../headers/Utils.hpp"
#include "../headers/Video.hpp"

namespace arfs
{

    void Camera::loadParameters(const std::string& filename)
    {
        std::ifstream file(filename);
        std::string line;
        int i = 0;
        std::array<double, 9> values{};

        while(std::getline(file, line))
        {
            if(line.empty() || line[0] == '#' || i >= 9) continue;

            auto line_split = arfs::Utils::split(line, ' ');

            if(line_split.size() != 3) continue;

            try
            {
                values[i] = std::stod(line_split[0]);
                values[i + 1] = std::stod(line_split[1]);
                values[i + 2] = std::stod(line_split[2]);
                i += 3;
            }
            catch(const std::exception& e)
            {
                std::cout << e.what() << std::endl;
                continue;
            }
        }

        m_intrinsicParameters = cv::Matx33d(values.data());
    }

    void Camera::calibrateAndSave(const std::string& filename, const std::array<int, 2>& checkerBoardSize,
                                  const std::string& imgFolder, double resizeFactor,
                                  arfs::Video cap, bool needToTakePictures)
    {
        cv::Mat frame;
        if(needToTakePictures)
        {
            int cpt = 1;
            for(;;)
            {
                frame = cap.getNextFrame();

                int key = cv::waitKey(30);

                if(frame.empty() || key == 27)
                    break;

                cv::imshow("cam", frame);

                if(key == 's')
                {
                    const std::string filename_img = imgFolder + "img_" + std::to_string(cpt++) + ".jpg";
                    std::cout << "Saved : " << filename_img << std::endl;
                    cv::imwrite(filename_img, frame);
                }
            }

            cv::destroyWindow("cam");
        }

        // Based on https://www.learnopencv.com/camera-calibration-using-opencv/

        // Creating vector to store vectors of 3D points for each checkerboard image
        std::vector<std::vector<cv::Point3f> > objPoints;

        // Creating vector to store vectors of 2D points for each checkerboard image
        std::vector<std::vector<cv::Point2f> > imgPoints;

        // Defining the world coordinates for 3D points
        std::vector<cv::Point3f> objp;
        for(int i = 0; i < checkerBoardSize[1]; i++)
        {
            for(int j = 0; j < checkerBoardSize[0]; j++)
                objp.emplace_back(j, i, 0);
        }


        std::vector<cv::String> images;
        std::string path = imgFolder + "*.jpg";

        cv::glob(path, images);

        cv::Mat gray;

        std::vector<cv::Point2f> corner_pts;
        bool success;

        for(auto& image : images)
        {
            frame = cv::imread(image);
            cv::resize(frame, frame, cv::Size(), resizeFactor, resizeFactor);
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            success = cv::findChessboardCorners(gray, cv::Size(checkerBoardSize[0], checkerBoardSize[1]), corner_pts,
                                                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
                                                cv::CALIB_CB_NORMALIZE_IMAGE);

            if(success)
            {
                cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

                // refining pixel coordinates for given 2d points.
                cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

                cv::drawChessboardCorners(frame, cv::Size(checkerBoardSize[0], checkerBoardSize[1]), corner_pts, success);

                objPoints.push_back(objp);
                imgPoints.push_back(corner_pts);
            }

            cv::imshow("Image", frame);
        }

        cv::destroyWindow("Image");

        cv::Mat distCoeffs, R, T;

        cv::calibrateCamera(objPoints, imgPoints, cv::Size(gray.rows, gray.cols), m_intrinsicParameters, distCoeffs, R, T);

        std::cout << "cameraMatrix : " << m_intrinsicParameters << std::endl;

        saveParametersToFile(filename);
    }

    void Camera::saveParametersToFile(const std::string& filename)
    {
        std::ofstream file(filename);

        file << "# Intrinsic matrix for camera" << std::endl;

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                file << m_intrinsicParameters.row(i).col(j).val[0];
                if(j != 2)
                    file << " ";
            }
            file << std::endl;
        }
    }

    /**
     * Apply a gaussian elimination on a matrix
     * @param matrix matrix to solve
     * @param rows rows count on matrix
     * @param cols cols count on matrix
     */
    void Camera::gaussJordanElimination(cv::Mat &matrix, int rows, int cols)
    {
        //Pivot initialization
        int h = 0;
        int k = 0;

        while(h < rows && k < cols)
        {
            /* Find the k-th pivot */
            int i_max = h;
            for(int i = h+1; i < rows; i++)
            {
                if(fabs(matrix.at<double>(i, k)) > fabs(matrix.at<double>(i_max, k)))
                {
                    i_max = i;
                }
            }
            if(matrix.at<double>(i_max, k) == 0)
            {
                // No pivot, jump this column
                k = k+1;
            }
            else
            {
                if(i_max != h)
                {
                    for(int l = 0; l<cols; l++)
                    {
                        double save = matrix.at<double>(h, l);
                        matrix.at<double>(h, l) = matrix.at<double>(i_max, l);
                        matrix.at<double>(i_max, l) = save;
                    }
                }

                double norm = matrix.at<double>(h, k);
                for(int l = 0; l < cols; l++)
                {
                    matrix.at<double>(h, l) /= norm;
                }

                for(int i = h+1; i<rows; i++)
                {
                    double f = matrix.at<double>(i, k) / matrix.at<double>(h, k);
                    for(int j = k+1; j<cols; j++)
                    {
                        matrix.at<double>(i, j) -= (matrix.at<double>(h, j)) * f;
                    }
                }

                h = h+1;
                k = k+1;
            }

        }

        //Back substitution, to transform the matrix in row echelon form in a system
        for(int i = rows-2; i >= 0; i--)
        {
            for(int j = i+1; j<cols-1; j++)
            {
                matrix.at<double>(i, rows) -= matrix.at<double>(i, j) * matrix.at<double>(j, rows);
            }
        }
    }

    /**
     * Estimate the homography matrix between two sets of four points
     * @param tagPoints
     * @param dstPoints
     * @param matrix output matrix
     */
    void Camera::estimateHomography(std::vector<cv::Point_<int>> tagPoints, std::vector<cv::Point_<int>> dstPoints, cv::Mat matrix)
    {
        double data[72] = {double(-tagPoints[0].x), double(-tagPoints[0].y), -1.0,   0.0,   0.0,  0.0, double(tagPoints[0].x) * double(dstPoints[0].x), double(tagPoints[0].y) * double(dstPoints[0].x), double(-dstPoints[0].x),
                          0.0, 0.0,  0.0, double(-tagPoints[0].x), double(-tagPoints[0].y), -1.0, double(tagPoints[0].x) * double(dstPoints[0].y), double(tagPoints[0].y) * double(dstPoints[0].y), double(-dstPoints[0].y),
                        double(-tagPoints[1].x), double(-tagPoints[1].y), -1.0,   0.0,   0.0,  0.0, double(tagPoints[1].x) * double(dstPoints[1].x), double(tagPoints[1].y) * double(dstPoints[1].x), double(-dstPoints[1].x),
                          0.0,   0.0,  0.0, double(-tagPoints[1].x), double(-tagPoints[1].y), -1.0, double(tagPoints[1].x) * double(dstPoints[1].y), double(tagPoints[1].y) * double(dstPoints[1].y), double(-dstPoints[1].y),
                        double(-tagPoints[2].x), double(-tagPoints[2].y), -1.0,   0.0,   0.0,  0.0, double(tagPoints[2].x) * double(dstPoints[2].x), double(tagPoints[2].y) * double(dstPoints[2].x), double(-dstPoints[2].x),
                          0.0,   0.0,  0.0, double(-tagPoints[2].x), double(-tagPoints[2].y), -1.0, double(tagPoints[2].x) * double(dstPoints[2].y), double(tagPoints[2].y) * double(dstPoints[2].y), double(-dstPoints[2].y),
                        double(-tagPoints[3].x), double(-tagPoints[3].y), -1.0,   0.0,   0.0,  0.0, double(tagPoints[3].x) * double(dstPoints[3].x), double(tagPoints[3].y) * double(dstPoints[3].x), double(-dstPoints[3].x),
                          0.0,   0.0,  0.0, double(-tagPoints[3].x), double(-tagPoints[3].y), -1.0, double(tagPoints[3].x) * double(dstPoints[3].y), double(tagPoints[3].y) * double(dstPoints[3].y), double(-dstPoints[3].y), };

        cv::Mat homographicMatrix = cv::Mat(8, 9, CV_64F, data);

        gaussJordanElimination(homographicMatrix, 8,  9);

        matrix.at<double>(0, 0) = homographicMatrix.at<double>(0, 8);
        matrix.at<double>(0, 1) = homographicMatrix.at<double>(1, 8);
        matrix.at<double>(0, 2) = homographicMatrix.at<double>(2, 8);
        matrix.at<double>(1, 0) = homographicMatrix.at<double>(3, 8);
        matrix.at<double>(1, 1) = homographicMatrix.at<double>(4, 8);
        matrix.at<double>(1, 2) = homographicMatrix.at<double>(5, 8);
        matrix.at<double>(2, 0) = homographicMatrix.at<double>(6, 8);
        matrix.at<double>(2, 1) = homographicMatrix.at<double>(7, 8);
        matrix.at<double>(2, 2) = 1;
    }


    void Camera::updateProjectionMatrix(const std::vector<cv::Point>& tagPoints)
    {
        m_projectionMatrix = cv::Mat();
        if(tagPoints.size() != 4) return;

        auto dstPoints = std::vector<cv::Point>{cv::Point(0, 0),
                                                cv::Point(m_tagProjectionSize, 0),
                                                cv::Point(m_tagProjectionSize, m_tagProjectionSize),
                                                cv::Point(0, m_tagProjectionSize)};


        cv::Mat homography_inv = cv::Mat(3, 3, CV_64F);
        estimateHomography(tagPoints, dstPoints, homography_inv);
        homography_inv = homography_inv.inv();

        if(homography_inv.empty()) return;

        auto R_and_T = m_intrinsicParameters.inv() * homography_inv;
        auto r1 = R_and_T.col(0);
        auto r2 = R_and_T.col(1);
        auto T = R_and_T.col(2);
        auto norm = sqrt(cv::norm(r1) * cv::norm(r2));

        r1 /= norm;
        r2 /= norm;
        T /= norm;
        auto c = r1 + r2;
        auto p = r1.cross(r2);
        auto d = c.cross(p);
        r1 = (c / cv::norm(c) + d / cv::norm(d)) * (1 / sqrt(2));
        r2 = (c / cv::norm(c) - d / cv::norm(d)) * (1 / sqrt(2));
        auto r3 = r1.cross(r2);

        cv::Mat R_T;
        cv::Mat array[] = {r1, r2, r3, T};
        cv::hconcat(array, 4, R_T);

        m_projectionMatrix = m_intrinsicParameters * R_T;
    }
}
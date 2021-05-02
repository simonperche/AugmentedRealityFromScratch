# AugmentedRealityFromScratch
 
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/96fba42254024bf983cedd22ba061936)](https://www.codacy.com/gh/Solidras/AugmentedRealityFromScratch/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=Solidras/AugmentedRealityFromScratch&amp;utm_campaign=Badge_Grade)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

An augmented reality application from scratch using OpenCV for learning purpose.

## Description

This project was developed in collaboration with 
[Nicolas Lépy](https://github.com/nicolasLepy) as a school project for image processing and computer vision courses. The main idea was to realize an real time augmented reality application 'from scratch'. OpenCV was only used for containers (e.g. cv::Mat) and image retrieval from videos or webcams. 

The core of the project is based on the detection of ArUco tag in the image. The homography is then computed using intrinsic camera parameters and the objects (loaded from an .obj with an optional material .mtl) are projected onto the tag. 

Afterwards, a tracking solution has been added to improve the stabilization of the tag. It comes from OpenCV for time reasons. 

## Results

The application works in real time with standard webcam (OpenCV supports). This is an example of the sample scene from the 'resources/example.scene' file.

![](readme_files/ARFS.gif)

On the left, green borders are the detected tag, on the right, the final results. 

## Tasks
| Feature                                  | Progress     |
|------------------------------------------|--------------|
| Recognize ARTag                          | Done         |
| Find camera position                     | Done         |
| Project 3D objects                       | Done         |
| Add normals support                      | Done         |
| Add camera calibration                   | Done         |
| Add materials support (only flat color)  | Done         |
| Stabilize detected tag                   | Done         |
| Improve the robustness of tag detection  | Done         |


## Installation
### Dependencies
You will need :
* [OpenCV](https://opencv.org) (>= version 4)
* [OpenCV Contrib](https://github.com/opencv/opencv_contrib) with at least *tracking* module

You can either compile OpenCV or get it from pre-compiled version.

### Build
This project was tested with :
  * MSVC version 16 (2019)
  * OpenCV 4.5.0 with dynamic linking
  
## Contributors

[Simon Perche](https://github.com/Solidras) \
[Nicolas Lépy](https://github.com/nicolasLepy)


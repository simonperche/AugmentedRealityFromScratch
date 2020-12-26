# AugmentedRealityFromScratch
 
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/96fba42254024bf983cedd22ba061936)](https://www.codacy.com/gh/Solidras/AugmentedRealityFromScratch/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=Solidras/AugmentedRealityFromScratch&amp;utm_campaign=Badge_Grade)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

An augmented reality application from scratch using OpenCV for learning purpose.

## Tasks
| Feature                                  | Progress     |
|------------------------------------------|--------------|
| Recognize ARTag                          | Done         |
| Find camera position                     | Done         |
| Project 3D objects                       | Done         |
| Add normals support                      | Done         |
| Add camera calibration                   | Done         |
| Add materials support (only flat color)  | To do        |
|                                          |              |
| Stabilize detected tag                   | In dev       |
|                                          |              |
| Improve the robustness of tag detection  | To do        |


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

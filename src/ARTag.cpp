//
// Created by Simon on 22/12/2020.
//

#include <utility>

#include "../headers/ARTag.hpp"
#include "../headers/Utils.hpp"

namespace arfs
{
    ARTag::ARTag(const std::string& filename) : m_img(arfs::Utils::loadImage(filename))
    {
        generateCode();
    }

    ARTag::ARTag(cv::Mat img) : m_img(std::move(img))
    {
        generateCode();
    }

    void ARTag::generateCode()
    {
        // 8 is fixed because aruco tags work with 8x8 square
        unsigned int width = m_img.cols / 8;

        for(unsigned int i = 0; i < 8; i++)
        {
            for(unsigned int j = 0; j < 8; j++)
            {
                auto rect = cv::Rect(j * width, i * width, width, width);
                cv::Scalar mean = cv::mean(m_img(rect));
                m_code[j + i * 8] = (mean[0] < 128) ? 0 : 1;
            }
        }
    }
}
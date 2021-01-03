//
// Created by Simon on 22/12/2020.
//

#ifndef AR_ARTAG_HPP
#define AR_ARTAG_HPP

#include <string>
#include <array>
#include <opencv2/core.hpp>

namespace arfs
{
    /**
     * @brief An ARTag containing a 64-bit code (like ArUco tag).
     * @details This class permits to get the code of an ARTag using an image. It may be loaded from a file or a cv::Mat.
     */
    class ARTag
    {
    public:
        /**
         * @brief Construct ARTag using filename as image.
         * @param filename saved image
         */
        explicit ARTag(const std::string& filename);

        /**
         * @brief Construct ARTag using img as image.
         * @param img
         */
        explicit ARTag(cv::Mat img);

        std::array<int, 64> getCode() const
        { return m_code; };

        friend std::ostream& operator<<(std::ostream& os, const ARTag& tag);

    private:
        std::array<int, 64> m_code{};
        cv::Mat m_img;

        void generateCode();
    };
}


#endif //AR_ARTAG_HPP

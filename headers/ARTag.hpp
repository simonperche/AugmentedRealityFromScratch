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
    class ARTag
    {
    public:
        explicit ARTag(const std::string& filename);

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

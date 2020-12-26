//
// Created by Simon on 21/12/2020.
//

#ifndef AR_OBJLOADER_HPP
#define AR_OBJLOADER_HPP

#include <string>
#include <opencv2/core/types.hpp>
#include "Object.hpp"

namespace arfs
{
    class OBJLoader
    {
    public:
        static arfs::Object load(const std::string& objFilename, const std::string& mtlFilename = {});
    };
}

#endif //AR_OBJLOADER_HPP

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
    /**
     * @brief Handmade .obj and .mtl parser.
     */
    class OBJLoader
    {
    public:
        /**
         * @brief Parse an .obj file with an optional .mtl to create an Object
         * @param objFilename filepath of .obj
         * @param mtlFilename filepath of .mtl (default "")
         * @return The new created object.
         */
        static arfs::Object load(const std::string& objFilename, const std::string& mtlFilename = {});
    };
}

#endif //AR_OBJLOADER_HPP

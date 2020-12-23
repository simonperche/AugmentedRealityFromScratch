//
// Created by Simon on 23/12/2020.
//

#ifndef AR_EXCEPTIONS_HPP
#define AR_EXCEPTIONS_HPP

#include <exception>

namespace arfs::exceptions
{
class EmptyProjectionMatrix : public std::exception
{
public:
    virtual const char* what() const noexcept
    {
        return "projection matrix is empty";
    }
};
}

#endif //AR_EXCEPTIONS_HPP

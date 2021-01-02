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
        virtual const char *what() const noexcept
        {
            return "projection matrix is empty";
        }
    };

    class BadCommandLineFormatting : public std::exception
    {
    public:
        explicit BadCommandLineFormatting(const std::string& arg, const std::string& endingMessage="")
        : m_message("bad command line formatting. Please check " + arg + "argument. " + endingMessage)
        {}

        virtual const char *what() const noexcept
        {

            return m_message.c_str();
        }

    private:
        std::string m_message;
    };
}

#endif //AR_EXCEPTIONS_HPP

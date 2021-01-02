//
// Created by Simon on 02/01/2021.
//

#include <sstream>

#include "CommandLineParser.hpp"

namespace arfs
{
    CommandLineParser::CommandLineParser(int argc, char *argv[]) : m_argv(argc-1)
    {
        for(int i = 0 ; i < argc - 1 ; ++i)
        {
            m_argv[i] = std::string(argv[i+1]);
        }
    }

    bool CommandLineParser::getFlagValue(const std::string& longFlag, const std::string& shortFlag)
    {
        bool ret{false};
        for(const auto& arg : m_argv)
        {
            if(arg == shortFlag || arg == longFlag)
            {
                ret = true;
                break;
            }
        }

        return ret;
    }

}


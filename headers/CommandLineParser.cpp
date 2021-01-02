//
// Created by Simon on 02/01/2021.
//

#include <sstream>

#include "CommandLineParser.hpp"
#include "exceptions.hpp"

namespace arfs
{
    CommandLineParser::CommandLineParser(int argc, char *argv[]) : m_argv(argc-1)
    {
        for(int i = 0 ; i < argc - 1 ; ++i)
        {
            m_argv[i] = std::string(argv[i+1]);
        }
    }

//    template<typename T>
//    bool CommandLineParser::getArgValue(const std::string& shortArg, const std::string& longArg, T& output)
//    {
//        std::string argValue{};
//        bool found{false};
//        for(auto it = m_argv.begin() ; it != m_argv.end() ; ++it)
//        {
//            if(*it == shortArg || *it == longArg)
//            {
//                if(it+1 != m_argv.end() && (it+1)->find("-") != 0)
//                {
//                    argValue = *(it + 1);
//                    found = true;
//                    break;
//                }
//                else
//                {
//                    throw arfs::exceptions::BadCommandLineFormatting(longArg);
//                }
//            }
//        }
//
//        std::istringstream ss(argValue);
////        try
////        {
//            ss >> output;
////        }
////        catch(std::exception& e)
////        {
////            throw arfs::exceptions::BadCommandLineFormatting(longArg, "Cannot convert " + argValue + " to the right type.");
////        }
//
//
//        return found;
//    }

    bool CommandLineParser::getFlagValue(const std::string& shortFlag, const std::string& longFlag)
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


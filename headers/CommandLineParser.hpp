//
// Created by Simon on 02/01/2021.
//

#ifndef AR_COMMANDLINEPARSER_HPP
#define AR_COMMANDLINEPARSER_HPP

#include <vector>
#include <string>
#include "exceptions.hpp"

namespace arfs
{
    class CommandLineParser
    {
    public:
        CommandLineParser(int argc, char *argv[]);

        template<typename T>
        bool getArgValue(const std::string& longArg, const std::string& shortArg, T& output)
        {
            std::string argValue{};
            bool found{false};
            for(auto it = m_argv.begin() ; it != m_argv.end() ; ++it)
            {
                if(*it == shortArg || *it == longArg)
                {
                    if(it+1 != m_argv.end() && (it+1)->find("-") != 0)
                    {
                        argValue = *(it + 1);
                        found = true;
                        break;
                    }
                    else
                    {
                        throw arfs::exceptions::BadCommandLineFormatting(longArg);
                    }
                }
            }

            if(found)
            {
                // Comes from https://gist.github.com/mark-d-holmberg/862733
                std::istringstream ss(argValue);
                ss >> output;
            }

            return found;
        }

        bool getFlagValue(const std::string& longFlag, const std::string& shortFlag);


    private:
        std::vector<std::string> m_argv;

    };

}

#endif //AR_COMMANDLINEPARSER_HPP

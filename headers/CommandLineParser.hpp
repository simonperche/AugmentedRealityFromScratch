//
// Created by Simon on 02/01/2021.
//

#ifndef AR_COMMANDLINEPARSER_HPP
#define AR_COMMANDLINEPARSER_HPP

#include <vector>
#include <string>

namespace arfs
{
    class CommandLineParser
    {
    public:
        CommandLineParser(int argc, char *argv[]);

        template<typename T>
        bool getArgValue(const std::string& shortArg, const std::string& longArg, T& output)
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

            std::istringstream ss(argValue);
//        try
//        {
            ss >> output;
//        }
//        catch(std::exception& e)
//        {
//            throw arfs::exceptions::BadCommandLineFormatting(longArg, "Cannot convert " + argValue + " to the right type.");
//        }


            return found;
        }

        bool getFlagValue(const std::string& shortFlag, const std::string& longFlag);


    private:
        std::vector<std::string> m_argv;

    };

}

#endif //AR_COMMANDLINEPARSER_HPP

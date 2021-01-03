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
    /**
     * @brief Handmade parser to handle command line parameters.
     */
    class CommandLineParser
    {
    public:
        /**
         * @brief Construct command line parser using parameters from main(argc, argv).
         * @param argc number of arguments
         * @param argv c array containing all arguments
         */
        CommandLineParser(int argc, char *argv[]);

        /**
         * @brief Query the parser to get value of a parameter.
         * @tparam T desired type of output
         * @param longArg expected long argument (e.g. --webcam 0)
         * @param shortArg expected short argument (e.g. -w 0)
         * @param[out] output reference of type T
         * @return true if argument is found, false otherwise
         */
        template<typename T>
        bool getArgValue(const std::string& longArg, const std::string& shortArg, T& output)
        {
            std::string argValue{};
            bool found{false};
            for(auto it = m_argv.begin(); it != m_argv.end(); ++it)
            {
                if(*it == shortArg || *it == longArg)
                {
                    if(it + 1 != m_argv.end() && (it + 1)->find("-") != 0)
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

        /**
         * @brief Query the parser to check if a flag is given or not.
         * @param longFlag expected long flag (e.g. --help)
         * @param shortFlag expected short flag (e.g. -h)
         * @return true if flag exists, false otherwise
         */
        bool getFlagValue(const std::string& longFlag, const std::string& shortFlag);


    private:
        std::vector<std::string> m_argv;

    };

}

#endif //AR_COMMANDLINEPARSER_HPP

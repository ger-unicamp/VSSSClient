#ifndef ARGPARSE
#define ARGPARSE
/**
 * @file argparse.h
 * @brief   Argument parser. Can use a long form (--argument) or short form (-a) 
 *          Usage:
 *            ArgParse::ArgumentParser parser(argv[0], "Testing");
 *          
 *            parser.add_argument('s', "string", "Argument 1", new ArgParse::value<std::string>("oi"));
 *            parser.add_argument('f', "float", "Argument 2", new ArgParse::value<float>("0.0"));
 *            parser.add_argument('b', "boolean", "Argument 3", new ArgParse::value<bool>("false"));
 *            parser.add_argument('g', "flag", "A flag");
 *            parser.add_argument('h', "help", "Help Menu");
 *            
 *            auto args = parser.parse_args(argc, argv);
 * 
 *            if(args["help"]->as<bool>())
 *            {
 *                std::cout << parser.help() << std::endl;
 *                return 0;
 *            }
 *            if(args.count("string") != 0)
 *                std::cout << args["string"]->as<std::string>() << std::endl;
 *            if(args.count("float") != 0)
 *                std::cout << args["float"]->as<float>() << std::endl;
 *            
 *            // Flags are always defined on dictionary (as true or false)
 *            std::cout << args["flag"]->as<bool>() << std::endl;
 * 
 * 
 * @version 0.1
 * @date 2020-07-12
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <sstream>
#include <stdexcept>
#include <typeinfo>
#include <vector>

#include <getopt.h>

namespace ArgParse
{

    /**
     * @brief Return the UPPERCASE string s
     * 
     * @param s 
     * @return std::string 
     */
    std::string toupper(const std::string &s)
    {
        std::string ret(s);
        std::for_each(ret.begin(), ret.end(), [](char &c) {
            c = _toupper(c);
        });
        return ret;
    }

    /**
     * @brief Return the lowercase string s
     * 
     * @param s 
     * @return std::string 
     */
    std::string tolower(const std::string &s)
    {
        std::string ret(s);
        std::for_each(ret.begin(), ret.end(), [](char &c) {
            c = _tolower(c);
        });
        return ret;
    }

    class value_iface
    {
    protected:
        std::string val;
    public:
        void set(const std::string &s)
        {
            this->val = s;
        }

        template <typename U>
        U as()
        {
            std::stringstream ss;
            if(typeid(U) == typeid(bool))
                ss << std::boolalpha << val;
            else
                ss << val;
            U ret;
            ss >> ret;
            return ret;
        }
        virtual ~value_iface() {}
    };

    template <typename T>
    class value : public value_iface
    {
    public:
        value(const T &_val)
        {
            std::stringstream ss;
            if(typeid(T) == typeid(bool))
                ss << std::boolalpha << _val;
            else
                ss << _val;
            ss >> this->val;
        }
    };

    class Option
    {
    public:
        char short_name;
        std::string long_name;
        std::string description;
        std::shared_ptr<value_iface> arg;

        Option(char _short_name, std::string _long_name, std::string _description, std::shared_ptr<value_iface> _arg) : short_name(_short_name),
                                                                                                                        long_name(_long_name),
                                                                                                                        description(_description),
                                                                                                                        arg(_arg){};
    };

    class ArgumentParserError : std::runtime_error
    {
    public:
        ArgumentParserError(const char *e) : std::runtime_error(e) {}
    };

    class ArgumentParser
    {
        std::string name;
        std::string description;
        std::vector<Option> options;
        std::map<std::string, std::shared_ptr<value_iface>> parsed_args;

    public:
        ArgumentParser(std::string name, std::string description = "")
        {
            this->name = name;
            this->description = description;
        }
        ~ArgumentParser() {}

        std::string help()
        {
            std::stringstream ss;
            ss << this->name << ": " << this->description << std::endl;
            ss << "Usage: " << this->name;
            for (size_t i = 0; i < this->options.size(); i++)
            {
                ss << " -" << this->options[i].short_name;
                ss << "\\--" << this->options[i].long_name;
                ss << " " << ((this->options[i].arg == NULL) ? "" : std::string(toupper(this->options[i].long_name)));
            }
            ss << std::endl
               << "Options:" << std::endl;
            for (size_t i = 0; i < this->options.size(); i++)
            {
                ss << "\t-" << this->options[i].short_name;
                ss << "\\--" << this->options[i].long_name;
                ss << " " << ((this->options[i].arg == NULL) ? "" : std::string(toupper(this->options[i].long_name)));
                ss << ": " << this->options[i].description << std::endl;
            }

            return ss.str();
        }
        /**
         * @brief 
         * 
         * @param short_arg shortname of option (-o)
         * @param arg longname of option (--option)
         * @param description help option description
         * @param val A ArgParse::value<type>.
         */
        template <typename T>
        void add_argument(char short_arg,
                          std::string arg,
                          std::string description,
                          const value<T> &val)
        {
            this->options.push_back(Option(short_arg, arg, description, std::make_shared<value<T>>(val)));
        }
        /**
         * @brief Add 
         * 
         * @param short_arg 
         * @param arg 
         * @param description 
         */
        void add_argument(char short_arg,
                          std::string arg,
                          std::string description)
        {
            this->options.push_back(Option(short_arg, arg, description, nullptr));
        }
        /**
         * @brief 
         * 
         * @param argc 
         * @param argv 
         * @return std::map<std::string, ArgParse::value<typename T>> 
         */
        std::map<std::string, std::shared_ptr<value_iface>> parse_args(int argc, char *argv[])
        {
            int opt;
            std::string optstring;
            struct option *optstruct = new option[this->options.size() + 1];
            for (size_t i = 0; i < this->options.size(); i++)
            {
                optstring.push_back(this->options[i].short_name);
                if (this->options[i].arg != NULL)
                {
                    optstring.push_back(':');
                    optstruct[i] = {this->options[i].long_name.c_str(), required_argument, 0, this->options[i].short_name};
                }
                else
                {
                    optstruct[i] = {this->options[i].long_name.c_str(), no_argument, 0, this->options[i].short_name};
                }
            }
            optstruct[this->options.size()] = {0, 0, 0, 0};

            while ((opt = getopt_long(argc, argv, optstring.c_str(), optstruct, NULL)) != -1)
            {
                if (opt == '?')
                {
                    std::cerr << this->help() << std::endl;
                    std::cerr << "Error parsing argument " << char(optopt) << std::endl;
                    throw ArgumentParserError("Error parsing argument");
                }
                else
                {
                    // Find an option witch opt char == short_name
                    std::vector<Option>::iterator optit = std::find_if(this->options.begin(), this->options.end(), [opt](const ArgParse::Option &o) { return o.short_name == opt; });
                    if (optit != this->options.end())
                    {
                        int index = optit - this->options.begin();
                        if (this->options[index].arg == NULL)
                            this->parsed_args[this->options[index].long_name] = std::make_shared<value<bool>>(value<bool>(true));
                        else
                        {
                            // Set argument value
                            std::string arg(optarg);
                            this->options[index].arg->set(arg);
                            this->parsed_args[this->options[index].long_name] = this->options[index].arg;
                        }
                    }
                    else
                    {
                        std::cerr << this->help() << std::endl;
                        std::cerr << "Unknown option " << std::string(argv[optind]) << std::endl;
                        throw ArgumentParserError("Unknown option");
                    }
                }
            }

            // Track undefined/not found flag options as false
            for (size_t i = 0; i < this->options.size(); i++)
            {
                if (this->options[i].arg == NULL && this->parsed_args.count(this->options[i].long_name) == 0)
                    this->parsed_args[this->options[i].long_name] = std::make_shared<value<bool>>(value<bool>(false));
                else if(this->parsed_args.count(this->options[i].long_name) == 0)
                    this->parsed_args[this->options[i].long_name] = this->options[i].arg;
            }

            delete[] optstruct;

            return this->parsed_args;
        }
    };

}; // namespace ArgParse

#endif
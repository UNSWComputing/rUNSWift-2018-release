#include <iostream>
#include <sstream>
#include "options.hpp"

po::variables_map parseOptions(int argc, char *argv[]){
    po::options_description desc("Options");
    desc.add_options()
        ("help,h", "show vatnao help")
        ("filename,f", po::value<std::string>()->required(), "the blackboard dump to read from")
        ("colour_cal_top,t", po::value<std::string>()->default_value(DEFAULT_TOP_NNMC_FILE), "the top nnmc file to read, defaults to nnmc in image")
        ("colour_cal_bot,b", po::value<std::string>()->default_value(DEFAULT_BOT_NNMC_FILE), "the bottom nnmc file to read, defaults to nnmc in image");

    po::variables_map vm;

    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << getHelp(desc);
            return NULL;
        }

        // Throw an error if there is an invalid argument
        po::notify(vm);

    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << getHelp(desc);
        return NULL;
    }

    return vm;
}

std::string getHelp(po::options_description desc){
    std::ostringstream help;
    help << "Vatnao - Visionary Accessible Testing" << std::endl << desc << std::endl;
    return help.str();
}

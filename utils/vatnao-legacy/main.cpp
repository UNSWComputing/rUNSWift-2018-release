#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>
#include <utils/Logger.hpp>

#include "../../robot/blackboard/Blackboard.hpp"
#include "../../robot/perception/vision/VisionAdapter.hpp"
#include "../../robot/perception/vision/VisionDefinitions.hpp"

#include "utils/options.hpp"

#include "ui/ui.hpp"
#include "app/appAdaptor.hpp"
#include "app/exceptions.hpp"
#include "options.hpp"

namespace po = boost::program_options;
using namespace std;
using namespace boost;

po::variables_map config;

void setGlobalConfig() {
    po::options_description config_file_options;
    populate_options(config_file_options);

    ifstream ifs;
    ifs.open("../runswift.cfg");
    store(parse_config_file(ifs, config_file_options), config);
    ifs.close();
    ifs.open("/etc/runswift/runswift.cfg");
    store(parse_config_file(ifs, config_file_options), config);
    ifs.close();
    po::notify(config);
}

void setup() {
    // Global config must be set before anything else used
    setGlobalConfig();
    // Initialise the logger so that it can be used by Blackboard
    // (Blackboard will throw a runtime error sometimes)
    Logger::init(config["debug.logpath"].as<string>(), config["debug.log"].as<string>(), true);
}

int main(int argc, char *argv[]) {
    std::cout << "Begin Setup" << std::endl;
    po::variables_map vatnaoOptions = parseOptions(argc, argv);
    setup();
    std::cout << "Setup complete" << std::endl;

    std::cout << "Begin Loading Dump" << std::endl;
    string path = vatnaoOptions["filename"].as<string>();
    try {
        // Note: this try catch has problems, namely, QT doesn't like it when we throw
        // exceptions without letting it know. As such, any time we throw an exception
        // we'll get this warning:
        //
        //     Qt has caught an exception thrown from an event handler. Throwing
        //     exceptions from an event handler is not supported in Qt. You must
        //     reimplement QApplication::notify() and catch all exceptions there.
        //
        // And Qt will crash. That's not really too much of a problem cause we were just
        // going to exit anyway, but it's not clean.
        //
        // SamCB will fix this some day.
        AppAdaptor vatnaoApp = AppAdaptor(path);
        std::cout << "Dump Loaded" << std::endl;

        UI ui(argc, argv, &vatnaoApp);
        return ui.exec();
    } catch (InvalidDumpFileError& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    std::cerr << "Unexpected return" << std::endl;
    return 1;
}


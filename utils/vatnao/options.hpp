#include <string>
#include <boost/program_options.hpp>

//#define DEFAULT_NNMC_FILE "./image/home/nao/data/green_yuv_classifier.nnmc.bz2"
#define DEFAULT_TOP_NNMC_FILE "./image/home/nao/data/top.nnmc.bz2"
#define DEFAULT_BOT_NNMC_FILE "./image/home/nao/data/bot.nnmc.bz2"

namespace po = boost::program_options;

po::variables_map parseOptions(int argc, char *argv[]);
std::string getHelp(po::options_description desc);

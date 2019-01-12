#include "dumpParser.hpp"
#include <fstream>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/archive/binary_iarchive.hpp>

using namespace std;

DumpParser::DumpParser(const std::string fileName): naoData() {
    // Global flag to let runswift know it's running on vatnao
    vatNao = true;

    std::ifstream ifs;
    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;

    // catch if file not found
    ifs.exceptions(ifstream::eofbit | ifstream::failbit | ifstream::badbit);
    ifs.open(fileName.c_str(), ios::in | ios::binary);
    in.push(ifs);
    boost::archive::binary_iarchive inputArchive(in);

    std::cout << "Write Archive" << std::endl;
    inputArchive & naoData;
    std::cout << "Archive fully Loaded" << std::endl;
    // NaoData will sometimes load input with the index on the last frame, set it to
    // 0 just to be useful.
    naoData.setCurrentFrame(0);
    std::cout << "CurrFrame: " << currFrameIndex() << "/" << numFrames() << std::endl;
}

void DumpParser::writePrevFrame(Blackboard *blackboard){
    naoData.prevFrame();
    writeCurrFrame(blackboard);
}

void DumpParser::writeNextFrame(Blackboard *blackboard){
    naoData.nextFrame();
    writeCurrFrame(blackboard);
}

void DumpParser::writeCurrFrame(Blackboard *blackboard){
    Frame frame = naoData.getCurrentFrame();
    *blackboard = *frame.blackboard;
    std::cout << "Wrote Frame: " << currFrameIndex() << "/" << numFrames() << " to blackboard" << std::endl;
}

int DumpParser::numFrames(){
    return naoData.getFramesTotal();
}

int DumpParser::currFrameIndex(){
    return naoData.getCurrentFrameIndex();
}

bool DumpParser::isFinalFrame() {
    return currFrameIndex() + 1 == numFrames();
}

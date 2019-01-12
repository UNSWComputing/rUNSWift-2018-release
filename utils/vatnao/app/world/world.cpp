#include "world.hpp"
#include <iostream>

#include "../exceptions.hpp"

World::World(DumpParser *dump){
    dumpParser = dump;
    blackboard = new Blackboard();
    hasValidFrame = false;
}

int World::forward(int numFrames){
    int movedFrames = 0;
    while(movedFrames < numFrames && dumpParser->currFrameIndex() < dumpParser->numFrames()) {
        // we need to make sure both our frames are not null, otherwise it'll crash when we
        //  try processing. Make sure the number of frames moved is recorded.
        do {
            dumpParser->writeNextFrame(blackboard);
            movedFrames++;
        } while((readFrom(vision, topFrame) == NULL || readFrom(vision, botFrame) == NULL) && dumpParser->isFinalFrame() == false);
    }
    if (dumpParser->isFinalFrame() && hasValidFrame == false) {
        throw NoRawImagesInDumpError();
    } else {
        hasValidFrame = true;
    }
    return movedFrames;
}

int World::back(int numFrames){
    int movedFrames = 0;
    while(movedFrames < numFrames && dumpParser->currFrameIndex() >= 0) {
        // we need to make sure both our frames are not null, otherwise it'll crash when we
        //  try processing. Make sure the number of frames moved is recorded.
        do {
            dumpParser->writePrevFrame(blackboard);
            movedFrames++;
        } while(readFrom(vision, topFrame) == NULL || readFrom(vision, botFrame) == NULL);
    }
    return movedFrames;
}


#pragma once

#include "blackboard/Blackboard.hpp"
#include "../../../offnao/naoData.hpp"

/*
 * Simple reader for reading through blackboards
 */
class DumpParser {
    public:
        /**
         * Setup the dump file to parse
         *
         * @param fileName the name of the dump file to read
         */
        DumpParser(const std::string fileName);

        /**
         * Roll back to the previous frame, read it and write to a blackboard
         *
         * @param blackboard the blackboard to write the frame to
         */
        void writePrevFrame(Blackboard *blackboard);

        /**
         * Progress to the next frame, read it and write to a blackboard
         *
         * @param blackboard the blackboard to write the frame to
         */
        void writeNextFrame(Blackboard *blackboard);

        /**
         * Read the current frame  and write to a blackboard
         *
         * @param blackboard the blackboard to write the frame to
         */
        void writeCurrFrame(Blackboard *blackboard);

        /**
         * The total number of frames in the file
         */
        int numFrames();

        /**
         * The index of the frame we are currently reading
         */
        int currFrameIndex();

        /**
         * True if the current frame is the last frame
         */
        bool isFinalFrame();
    private:
        /**
         * holds the frames once imported
         */
        NaoData naoData;
};

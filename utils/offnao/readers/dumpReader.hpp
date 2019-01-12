#pragma once
#include <QString>
#include <cstdio>
#include "readers/reader.hpp"
#include "perception/vision/other/YUV.hpp"
#define TOP_SIZE 1280*960*2
#define BOT_SIZE 640*480*2
/*
 *  Simple reader that reads in video photage from file.
 */
class DumpReader : public Reader {
   Q_OBJECT
   public:
      explicit DumpReader(QString filename);
      explicit DumpReader(QString filename, const NaoData &naoData);
      virtual void run();
   private:
      FILE *dumpFile;
   public slots:
      virtual void stopMediaTrigger();
};

#pragma once

#include "reader.hpp"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <fstream>
#include <QString>

#define BOOST_HAS_COMPRESSION false

/*
 *  Simple reader that reads in recorded dumps from file.
 */
class RecordReader : public Reader {
   Q_OBJECT
   public:
      /**
       * sets up necessary file descriptors for reading
       *
       * @param fileName the name of the file to read
       */
      explicit RecordReader(const QString &fileName);

      /**
       * sets up necessary file descriptors for reading
       *
       * @param fileName the name of the file to read
       * @param naoData the old naoData to append to
       */
      explicit RecordReader(const QString &fileName, const NaoData &naoData);

      /**
       * read in record and store in naoData
       *
       * @param fileName the name of the file to read
       * @param naoData to store data
       */
      static void read(const QString &fileName, NaoData &naoData);

      /**
       * helper function to write a file readable by this class
       *
       * @param fileName the name of the file to write
       * @param naoData the data to write
       */
      static void write(const QString &fileName, const NaoData &naoData);

      /**
       * helper function to write the whiteboard - a subset of the blackboard
       *
       * @param fileName the name of the file to write
       * @param naoData the data to write
       */
      static void writeWhiteboard(const QString &fileName, const NaoData &naoData);

      /**
       * helper function to create images and save them as BMPs
       *
       * @param frame the image as an array in yuv format
       * @param startRow the starting row position of region to extract
       * @param endRow the ending row position of the region to extract
       * @param startCol the starting col position of region to extract
       * @param endCol the ending col position of the region to extract
       * @param numFrameCol the number of cols in the frame.
       * @param fileName the name of the file to save to
       */
      static void createImage(const uint8_t *frame, const int startRow, const int endRow,
          const int startCol, const int endCol, const int numFrameCol, const QString fileName);

      /**
       * helper function to dump YUV images. This is used for training machine
       * learning algorithms using raw YUV values. Note: this is a custom format
       * file, where we use the first 8 byted for the dimensions, and the remaining
       * data as the YUV values in the given region.
       *
       * @param frame the image as an array in yuv format
       * @param startRow the starting row position of region to extract
       * @param endRow the ending row position of the region to extract
       * @param startCol the starting col position of region to extract
       * @param endCol the ending col position of the region to extract
       * @param numFrameCol the number of cols in the frame.
       * @param fileName the name of the file to save to
       */
      static void createYUV(const uint8_t *frame, const int startRow, const int endRow,
          const int startCol, const int endCol, const int numFrameCol, const QString fileName);

      /**
       * function to dump raw images
       *
       * @param folderName the name of the folder to dump files to
       * @param naoData the data to write
       * @param extractROI extract regions of interest.
       */
      static void dumpImages(const QString &folderName, NaoData &naoData, bool extractROI);

      virtual void run();

      /**
       * closes appropriate file descriptors and frees memory
       */
      virtual ~RecordReader();

   private:
      const QString fileName;

};

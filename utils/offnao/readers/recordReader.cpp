#include "recordReader.hpp"

#include <boost/archive/binary_oarchive.hpp>
#if BOOST_HAS_COMPRESSION
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#endif
#include <sstream>

#include "tabs/classifier.hpp"
#include "blackboard/Blackboard.hpp"
#include "naoData.hpp"
#include "perception/vision/other/YUV.hpp"
#include "perception/vision/Region.hpp"
#include "types/BBox.hpp"

using namespace std;

RecordReader::RecordReader(const QString &fileName) : fileName(fileName)
{
}

RecordReader::RecordReader(const QString &fileName, const NaoData &naoData) :
Reader(naoData), fileName(fileName)
{
}

RecordReader::~RecordReader()
{
}

void RecordReader::read(const QString &fileName, NaoData &naoData)
{
   std::ifstream ifs;
   boost::iostreams::filtering_streambuf<boost::iostreams::input> in;

   // catch if file not found
   ifs.exceptions(ifstream::eofbit | ifstream::failbit | ifstream::badbit);
   ifs.open(qPrintable(fileName), ios::in | ios::binary);
   #if BOOST_HAS_COMPRESSION
   if (fileName.endsWith(".gz")) {
      in.push(boost::iostreams::gzip_decompressor());
   }
   if (fileName.endsWith(".bz2")) {
      in.push(boost::iostreams::bzip2_decompressor());
   }
   #endif
   in.push(ifs);
   boost::archive::binary_iarchive ia(in);

   NaoData nd;
   ia & nd;
   for(int f = 0; f < nd.getFramesTotal(); ++f) {
      naoData.appendFrame(nd.getFrame(f));
   }
}

void RecordReader::writeWhiteboard(const QString &fileName, const NaoData &naoData)
{
   /* Set the blackboard mask to whiteboard. This prevents some of the data
   being serialised to allow vision to be reporcessed and new training data to be
   collected.
   */
   naoData.setWhiteboardMask();
   ofstream ofs(qPrintable(fileName), ios::out | ios::binary | ios::trunc);
   boost::iostreams::filtering_streambuf<boost::iostreams::output> out;
   out.push(ofs);
   boost::archive::binary_oarchive oa(out);
   oa & naoData;
   naoData.removeWhiteboardMask();

}

void RecordReader::write(const QString &fileName, const NaoData &naoData)
{
   ofstream ofs(qPrintable(fileName), ios::out | ios::binary | ios::trunc);
   boost::iostreams::filtering_streambuf<boost::iostreams::output> out;
   #if BOOST_HAS_COMPRESSION
   if (fileName.endsWith(".gz"))
      out.push(boost::iostreams::gzip_compressor());
   if (fileName.endsWith(".bz2"))
      out.push(boost::iostreams::bzip2_compressor());
   #endif
   out.push(ofs);
   boost::archive::binary_oarchive oa(out);
   oa & naoData;
}

void RecordReader::createYUV(const uint8_t *frame, const int startRow, const int endRow,
    const int startCol, const int endCol, const int numFrameCols, const QString fileName)
{
      int imgHeight = endRow - startRow;
      int imgWidth = endCol - startCol;
      cout << "YUV dump Height x Width: "<< imgHeight << "x" << imgWidth <<endl;
      uint8_t image[imgHeight*imgWidth*3];

      // bmp rows stored backwards
      for (int i = 0; i < imgHeight; ++i) {
         for (int j = 0; j < imgWidth; ++j) {
            int x = j;
            int y = (imgHeight-1)-i;

            // Construct the array that contains the raw YUV values.
            image[(((y*imgWidth) + x)*3)+0] = gety(frame, startRow + i, startCol + j, numFrameCols);
            image[(((y*imgWidth) + x)*3)+1] = getu(frame, startRow + i, startCol + j, numFrameCols);
            image[(((y*imgWidth) + x)*3)+2] = getv(frame, startRow + i, startCol + j, numFrameCols);
         }
      }
      FILE *dumpFile = fopen(fileName.toUtf8().constData(), "w");

      // TODO: Check if this is the right thing in the python trainer??
      // Encode the image dimensions into the file
      unsigned int fileHeader[2] = {imgHeight, imgWidth};

      // Write header info
      fwrite(fileHeader,4,2,dumpFile);

      // Write image data
      fwrite(&image, imgHeight*imgWidth*3, 1, dumpFile);
      fflush(dumpFile);
      fclose(dumpFile);
}


void RecordReader::createImage(const uint8_t *frame, const int startRow, const int endRow,
    const int startCol, const int endCol, const int numFrameCols, const QString fileName)
{
      int imgHeight = endRow - startRow;
      int imgWidth = endCol - startCol;
      cout << "Height x Width: "<< imgHeight << "x" << imgWidth <<endl;
      uint8_t image[imgHeight*imgWidth*3];

      cout<<"Creating an image" << endl;
      // bmp rows stored backwards
      for (int i = 0; i < imgHeight; ++i) {
         for (int j = 0; j < imgWidth; ++j) {
            int x = j;
            int y = (imgHeight-1)-i;

            // Get the pixel from the frame, offset by the starting row/col.
            QRgb rgb = Classifier::yuv2rgb(gety(frame, startRow + i, startCol + j, numFrameCols),
                           getu(frame, startRow + i, startCol + j, numFrameCols),
                           getv(frame, startRow + i, startCol + j, numFrameCols));

            // Construct the array that contains the raw RGB values.
            image[(((y*imgWidth) + x)*3)+0] = qBlue(rgb);
            image[(((y*imgWidth) + x)*3)+1] = qGreen(rgb);
            image[(((y*imgWidth) + x)*3)+2] = qRed(rgb);
         }
      }
      FILE *dumpFile = fopen(fileName.toUtf8().constData(), "w");

      // Thanks stack overflow
      // http://stackoverflow.com/questions/2654480/writing-bmp-image-in-pure-c-c-without-other-libraries

      int filesize = 54 + (3*imgHeight*imgWidth);

      // Little endian

      // Set header info
      unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
      unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};

      bmpfileheader[ 2] = (unsigned char)(filesize    );
      bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
      bmpfileheader[ 4] = (unsigned char)(filesize>>16);
      bmpfileheader[ 5] = (unsigned char)(filesize>>24);

      bmpinfoheader[ 4] = (unsigned char)(       imgWidth    );
      bmpinfoheader[ 5] = (unsigned char)(       imgWidth>> 8);
      bmpinfoheader[ 6] = (unsigned char)(       imgWidth>>16);
      bmpinfoheader[ 7] = (unsigned char)(       imgWidth>>24);
      bmpinfoheader[ 8] = (unsigned char)(       imgHeight    );
      bmpinfoheader[ 9] = (unsigned char)(       imgHeight>> 8);
      bmpinfoheader[10] = (unsigned char)(       imgHeight>>16);
      bmpinfoheader[11] = (unsigned char)(       imgHeight>>24);

      // Write header info
      fwrite(bmpfileheader,1,14,dumpFile);
      fwrite(bmpinfoheader,1,40,dumpFile);

      // Write image data
      fwrite(&image, imgHeight*imgWidth*3, 1, dumpFile);
      fflush(dumpFile);
      fclose(dumpFile);
}

void RecordReader::dumpImages(const QString &folderName, NaoData &naoData, bool extractROI)
{

   // Save current frame so it can be restored
   int currFrame = naoData.getCurrentFrameIndex();

   // Start from the beginning
   naoData.setCurrentFrame(0);

   Blackboard *blackboard;

   uint8_t const* topFrame;
   uint8_t const* botFrame;
   int prev = -1;
   while (prev != naoData.getCurrentFrameIndex()) {

      cout << "Extractnig" ;

      blackboard = naoData.getCurrentFrame().blackboard;

      topFrame = readFrom(vision, topFrame);
      botFrame = readFrom(vision, botFrame);

      if (topFrame == NULL || botFrame == NULL) {
         prev = naoData.getCurrentFrameIndex();
         naoData.nextFrame();
         continue;
      }

      // Create the images
      if (extractROI) {
          //TODO: Read the region array from the blackboard. Iterate for each bounding box and do the following.
         // Extract the regions of interest only.
         std::vector<RegionI> bb_regions = readFrom(vision, regions);
         cout << "Num Regions " << bb_regions.size();
         for (unsigned int i=0; i < bb_regions.size(); i++){
             BBox b_box = bb_regions[i].getBoundingBoxRaw();
             cout << "Region from x:" << b_box.a.x() << " to " << b_box.b.x() << endl;
             cout << "Region from y:" << b_box.a.y() << " to " << b_box.b.y() << endl;

             if (bb_regions[i].isTopCamera()){
                 cout << "Creating new something"<<endl;
                 createImage(topFrame,
                             b_box.a.y() , b_box.b.y() ,
                             b_box.a.x() , b_box.b.x() ,
                             TOP_IMAGE_COLS, folderName + "/top_image_" +  QString::number(naoData.getCurrentFrameIndex()) +
                             "_roi_" + QString::number(i) + ".bmp");
                 // While we are at it, we also create a YUV image.
                 createYUV(topFrame,
                             b_box.a.y(), b_box.b.y(),
                             b_box.a.x(), b_box.b.x(),
                             TOP_IMAGE_COLS, folderName + "/top_image_" +  QString::number(naoData.getCurrentFrameIndex()) +
                             "_roi_" + QString::number(i) + ".yuv");
             } else {
                 cout << "Bottom extraction"<<endl;
                 createImage(botFrame,
                             b_box.a.y() , b_box.b.y(),
                             b_box.a.x() , b_box.b.x(),
                             BOT_IMAGE_COLS, folderName + "/bot_image_" +  QString::number(naoData.getCurrentFrameIndex()) +
                             "_roi_" + QString::number(i) + ".bmp");
                 // While we are at it, we also create a YUV image.
                 createYUV(botFrame,
                             b_box.a.y(), b_box.b.y(),
                             b_box.a.x(), b_box.b.x(),
                             BOT_IMAGE_COLS, folderName + "/bot_image_" +  QString::number(naoData.getCurrentFrameIndex()) +
                             "_roi_" + QString::number(i) + ".yuv");
             }
         }
      } else {
         // Extract the full images.
         createImage(topFrame, 0, TOP_IMAGE_ROWS, 0, TOP_IMAGE_COLS, TOP_IMAGE_COLS,
                  folderName + "/top_image_" + QString::number(naoData.getCurrentFrameIndex()) + ".bmp");

         createImage(botFrame, 0, BOT_IMAGE_ROWS, 0, BOT_IMAGE_COLS, BOT_IMAGE_COLS,
                  folderName + "/bot_image_" + QString::number(naoData.getCurrentFrameIndex()) + ".bmp");
         createYUV(botFrame, 0, BOT_IMAGE_ROWS, 0, TOP_IMAGE_COLS, TOP_IMAGE_COLS,
                  folderName + "/top_image_" + QString::number(naoData.getCurrentFrameIndex()) + ".yuv");
         createYUV(botFrame, 0, BOT_IMAGE_ROWS, 0, BOT_IMAGE_COLS, BOT_IMAGE_COLS,
                  folderName + "/bot_image_" + QString::number(naoData.getCurrentFrameIndex()) + ".yuv");
      }

      // Break if the next frame is the same as the current frame
      prev = naoData.getCurrentFrameIndex();
      naoData.nextFrame();
   }

   // Restore frame
   naoData.setCurrentFrame(currFrame);
}

void RecordReader::run() {
   try {
      read(fileName, naoData);
   } catch(const std::exception& e) {
      QString s("Can not load record: ");
      emit disconnectFromNao();
      emit showMessage(s + e.what());
      emit openFile();
      return;
   }
   stringstream s;
   s << "Finished loading record which consisted of " <<
        naoData.getFramesTotal() << " frames.";
   emit showMessage(s.str().c_str(), 5000);
   emit newNaoData(&naoData);

   int currentIndex = 0;
   isAlive = true;
   while (isAlive) {
      if (!naoData.getIsPaused() &&
         naoData.getCurrentFrameIndex() < naoData.getFramesTotal() - 1) {
         naoData.nextFrame();
         emit newNaoData(&naoData);
      } else if (currentIndex != naoData.getCurrentFrameIndex()) {
         emit newNaoData(&naoData);
      }
      currentIndex = naoData.getCurrentFrameIndex();
      msleep(500);
   }
   emit newNaoData(NULL);
}

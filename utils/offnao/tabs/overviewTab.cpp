#include <QMenu>
#include <QMenuBar>
#include <QDebug>
#include <QBitmap>
#include <QFileDialog>

#include <utility>
#include <iostream>
#include "overviewTab.hpp"
#include "perception/vision/other/YUV.hpp"
#include "perception/kinematics/Pose.hpp"
#include "blackboard/Blackboard.hpp"

#include "utils/CPlaneColours.hpp"

#include "perception/vision/Vision.hpp"

#include "perception/vision/Region.hpp"

#define DEBUG_IMAGE_ROWS (IMAGE_ROWS / 4)
#define DEBUG_IMAGE_COLS (IMAGE_COLS / 4)

using namespace std;

OverviewTab::OverviewTab(QTabWidget *parent, QMenuBar *menuBar,
      Vision *vision) : blackboard(0) {
   initMenu(menuBar);
   init();
   this->vision = vision;
   memset(topSaliency, 0, TOP_SALIENCY_ROWS*TOP_SALIENCY_COLS*sizeof(Colour));
   memset(botSaliency, 0, BOT_SALIENCY_ROWS*BOT_SALIENCY_COLS*sizeof(Colour));
   this->parent = parent;
}


void OverviewTab::initMenu(QMenuBar * menuBar) {
   vision2017Menu = new QMenu("Vision2017");
   menuBar->addMenu(vision2017Menu);

   loadTopNnmcAct = new QAction(tr("Load NNMC (top)"), vision2017Menu);
   vision2017Menu->addAction(loadTopNnmcAct);

   loadBotNnmcAct = new QAction(tr("Load NNMC (bot)"), vision2017Menu);
   vision2017Menu->addAction(loadBotNnmcAct);

   saveTopNNMCAct = new QAction(tr("Save robot's NNMC (top)"), vision2017Menu);
   vision2017Menu->addAction(saveTopNNMCAct);

   saveBotNNMCAct = new QAction(tr("Save robot's NNMC (bot)"), vision2017Menu);
   vision2017Menu->addAction(saveBotNNMCAct);

   // connect the actions
   connect(loadTopNnmcAct, SIGNAL(triggered()), this, SLOT(loadTopNnmc ()));
   connect(loadBotNnmcAct, SIGNAL(triggered()), this, SLOT(loadBotNnmc ()));
   connect(saveTopNNMCAct, SIGNAL(triggered()), this, SLOT(saveTopNNMC()));
   connect(saveBotNNMCAct, SIGNAL(triggered()), this, SLOT(saveBotNNMC()));
}

void OverviewTab::init() {
   layout = new QGridLayout(this);
   setLayout(layout);
   layout->setAlignment(layout, Qt::AlignTop);

   layout->setHorizontalSpacing(5);
   layout->setHorizontalSpacing(5);


   layout->addWidget(&fieldView, 0, 0, 2, 1);

   /* draw the field with nothing on it */
   fieldView.redraw(NULL);

   topImagePixmap = QPixmap(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   topImagePixmap.fill(Qt::darkGray);
   topCamLabel  = new QLabel();
   topCamLabel->setPixmap(topImagePixmap);
   topCamLabel->setMinimumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   topCamLabel->setMaximumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   layout->addWidget(topCamLabel, 0, 1, 1, 2);

   botImagePixmap = QPixmap(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   botImagePixmap.fill(Qt::darkGray);
   botCamLabel  = new QLabel();
   botCamLabel->setPixmap(botImagePixmap);
   botCamLabel->setMinimumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   botCamLabel->setMaximumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   layout->addWidget(botCamLabel, 1, 1, 1, 2);

   layout->addWidget(&variableView, 0, 3, 2, 2);
}

void OverviewTab::redraw() {
   if (topFrame || botFrame || topSaliency || botSaliency) {
      QImage *topImage;
      QImage *botImage;

      topImage = new QImage(TOP_SALIENCY_COLS,
          TOP_SALIENCY_ROWS,
          QImage::Format_RGB32);
      botImage = new QImage(BOT_SALIENCY_COLS,
          BOT_SALIENCY_ROWS,
          QImage::Format_RGB32);

      drawImage(topImage, botImage);

      // Scale iamges up to real size to draw overlays
      QPixmap t = QPixmap::fromImage(
               topImage->scaled(2*DEBUG_IMAGE_COLS, 2*DEBUG_IMAGE_ROWS));
      botImagePixmap = QPixmap::fromImage(
               botImage->scaled(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS));

      drawOverlays(&t, &botImagePixmap);

      // Rescale the top image back to 640x480 to fit the screen
      topImagePixmap = t.scaled(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);

      delete topImage;
      delete botImage;
   } else {
      topImagePixmap = QPixmap(IMAGE_COLS, IMAGE_ROWS);
      topImagePixmap.fill(Qt::darkRed);
      botImagePixmap = QPixmap(IMAGE_COLS, IMAGE_ROWS);
      botImagePixmap.fill(Qt::darkRed);
   }
   topCamLabel->setPixmap(topImagePixmap);
   botCamLabel->setPixmap(botImagePixmap);
}

void OverviewTab::drawOverlays(QPixmap *topImage, QPixmap *botImage) {
   if (!blackboard) return;

   std::vector<FootInfo>	        feet_boxes	    = readFrom(vision, feet_boxes);
   std::vector<BallInfo>            balls           = readFrom(vision, balls);
   std::vector<PostInfo>            posts           = readFrom(vision, posts);
   std::vector<RobotInfo>           robots          = readFrom(vision, robots);
   std::vector<FieldBoundaryInfo>   fieldBoundaries = readFrom(vision, fieldBoundaries);
   std::vector<FieldFeatureInfo>    fieldFeatures   = readFrom(vision, fieldFeatures);
   std::vector<Ipoint>              landmarks       = readFrom(vision, landmarks);
   std::vector<RegionI>             regions         = readFrom(vision, regions);

   std::pair<int, int> horizon = readFrom(motion, pose).getHorizon();
   std::pair<int, int> *horizon_p = &horizon;

   drawOverlaysGeneric (topImage,
                        botImage,
                        horizon_p,
                        &feet_boxes,
                        &balls,
                        &posts,
                        &robots,
                        &fieldBoundaries,
                        &fieldFeatures,
                        &landmarks,
                        &regions,
                        0.5
                       );



   /*
   QPainter painter(image);
   const Pose &pose = readFrom(kinematics, pose);
   const std::pair<int, int> horizon = pose.getHorizon();
   painter.setBrush(QBrush(QColor(255, 255, 255)));
   painter.drawLine(0,horizon.first/SALIENCY_DENSITY*2,
         640/SALIENCY_DENSITY*2,
         horizon.second/SALIENCY_DENSITY*2);

   //draw body exclusion points
   painter.setBrush(QBrush(QColor(255, 255, 255)));
   float scale = 2.0/SALIENCY_DENSITY;
   const int16_t *points = pose.getExclusionArray();
   for (int i = 0; i < Pose::EXCLUSION_RESOLUTION; i++) {
       painter.drawEllipse(QPoint(scale*640 * i*1.0/Pose::EXCLUSION_RESOLUTION,
                         scale*points[i]), 2, 2);
   }
   */

   return;
}


void OverviewTab::drawImage(QImage *topImage, QImage *botImage) {
   if (topFrame && botFrame) {
       VisionInfoIn info_in;
       vision->processFrame(CombinedFrame(topFrame, botFrame), info_in);

       fieldView.redraw(NULL);

        // Top Image
       for (int row = 0; row < TOP_SALIENCY_ROWS; ++row) {
          for (int col = 0; col < TOP_SALIENCY_COLS; ++col) {
             topImage->setPixel(col, row,
                CPLANE_COLOURS[vision->getFullRegionTop().getPixelColour(col, row)].rgb());
          }
       }

       // Bottom Image
       for (int row = 0; row < BOT_SALIENCY_ROWS; ++row) {
          for (int col = 0; col < BOT_SALIENCY_COLS; ++col) {
             botImage->setPixel(col, row,
                CPLANE_COLOURS[vision->getFullRegionBot().getPixelColour(col, row)].rgb());
          }
       }
    } else {

     for (unsigned int row = 0;
           row < TOP_IMAGE_ROWS / TOP_SALIENCY_DENSITY; ++row) {
         for (unsigned int col = 0;
              col < TOP_IMAGE_COLS / TOP_SALIENCY_DENSITY; ++col) {
             if (0 <= topSaliency[row][col] && topSaliency[row][col] < cNUM_COLOURS) {
                topImage->setPixel(col, row,
                    CPLANE_COLOURS[topSaliency[row][col]].rgb());
             } else {
                std::cerr << "Bad pixel at " << row << " " << col << std::endl;
             }
         }
      }
      for (unsigned int row = 0;
           row < BOT_IMAGE_ROWS / BOT_SALIENCY_DENSITY; ++row) {
         for (unsigned int col= 0;
              col < BOT_IMAGE_COLS / BOT_SALIENCY_DENSITY; ++col) {
             if (0 <= botSaliency[row][col] && botSaliency[row][col] < cNUM_COLOURS) {
                 botImage->setPixel(col, row,
                     CPLANE_COLOURS[botSaliency[row][col]].rgb());
             } else {
                 std::cerr << "Bad pixel at " << row << " " << col << std::endl;
             }
         }
      }

    }
}

// TODO(brockw): see if this can be genericized into tab.cpp, so it's not in
// every tab
void OverviewTab::newNaoData(NaoData *naoData) {

   if (!naoData || !naoData->getCurrentFrame().blackboard) {  // clean up display, as read is finished
      topImagePixmap.fill(Qt::darkGray);
      topCamLabel->setPixmap(topImagePixmap);
      botImagePixmap.fill(Qt::darkGray);
      botCamLabel->setPixmap(botImagePixmap);
   } else if (naoData->getFramesTotal() != 0) {
      blackboard = (naoData->getCurrentFrame().blackboard);
      topFrame = readFrom(vision, topFrame);
      botFrame = readFrom(vision, botFrame);
      if (!topFrame) {
         if (readFrom(vision, topSaliency))
            memcpy(topSaliency, readFrom(vision, topSaliency),
                   TOP_SALIENCY_ROWS*TOP_SALIENCY_COLS*sizeof(Colour));
      }
      if (!botFrame) {
         if (readFrom(vision, botSaliency))
            memcpy(botSaliency, readFrom(vision, botSaliency),
                   BOT_SALIENCY_ROWS*BOT_SALIENCY_COLS*sizeof(Colour));
      }
      if (parent->currentIndex() == parent->indexOf(this)) {
         redraw();
         fieldView.redraw(naoData);
         variableView.redraw(naoData);
      }
   }
}

void OverviewTab::loadTopNnmc() {
    QString fileName = QFileDialog::getOpenFileName(this, "Load Top Nnmc File");
    if (fileName != NULL) {
        this->vision->loadNnmc(true, fileName.toStdString().c_str());
    }
}

void OverviewTab::loadBotNnmc() {
    QString fileName = QFileDialog::getOpenFileName(this, "Load Bot Nnmc File");
    if (fileName != NULL) {
        this->vision->loadNnmc(false, fileName.toStdString().c_str());
    }
}

void OverviewTab::saveTopNNMC() {
   QString fileName = QFileDialog::getSaveFileName(this, "Save Top Nnmc File");
   if (fileName != NULL) {
      //scpNNMC.start("scp nao@" + naoName + ":/home/nao/data/green_yuv_classifier_tmp.nnmc.bz2 " + fileName,
      scpNNMC.start("scp nao@" + naoName + ":/home/nao/data/top.nnmc.bz2 " + fileName,
         QIODevice::ReadOnly | QIODevice::Text);
   }
}

void OverviewTab::saveBotNNMC() {
   QString fileName = QFileDialog::getSaveFileName(this, "Save Bot Nnmc File");
   if (fileName != NULL) {
      //scpNNMC.start("scp nao@" + naoName + ":/home/nao/data/green_yuv_classifier_tmp.nnmc.bz2 " + fileName,
      scpNNMC.start("scp nao@" + naoName + ":/home/nao/data/bot.nnmc.bz2 " + fileName,
         QIODevice::ReadOnly | QIODevice::Text);
   }
}
void OverviewTab::setNao(const QString &naoName) {
   this->naoName = naoName;
}

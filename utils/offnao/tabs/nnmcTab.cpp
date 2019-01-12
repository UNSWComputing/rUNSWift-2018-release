#include <QMenu>
#include <QMenuBar>
#include <QDebug>
#include <QBitmap>
#include <QFileDialog>

#include <utility>
#include <iostream>
#include "nnmcTab.hpp"
#include "perception/vision/other/YUV.hpp"
#include "perception/kinematics/Pose.hpp"
#include "blackboard/Blackboard.hpp"

#include "utils/CPlaneColours.hpp"

#include "perception/vision/Vision.hpp"

#include "perception/vision/Region.hpp"

#define DEBUG_IMAGE_ROWS (IMAGE_ROWS / 4)
#define DEBUG_IMAGE_COLS (IMAGE_COLS / 4)

using namespace std;

NNMCTab::NNMCTab(QTabWidget *parent, QMenuBar *menuBar,
      Vision *vision) : blackboard(0) {
   initMenu(menuBar);
   init();
   memset(topSaliency, 0, TOP_SALIENCY_ROWS*TOP_SALIENCY_COLS*sizeof(Colour));
   memset(botSaliency, 0, BOT_SALIENCY_ROWS*BOT_SALIENCY_COLS*sizeof(Colour));
   this->parent = parent;

   this->vision = vision;
   nnmcLoaded = false;
   green_yuv_classifier_top_ = new GreenYUVClassifier(false, false, false, "");
   green_yuv_classifier_bot_ = new GreenYUVClassifier(false, false, false, "");
   haveData = false;
}

void NNMCTab::initMenu(QMenuBar * menuBar) {
}

void NNMCTab::init() {
   layout = new QGridLayout();
   this->setLayout(layout);

   layout->setAlignment(layout, Qt::AlignTop);

   layout->setHorizontalSpacing(5);

   /* Setup the images */

   topImageRawPixmap = QPixmap(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   topImageRawPixmap.fill(Qt::darkGray);
   topCamRawLabel  = new QLabel();
   topCamRawLabel->setPixmap(topImageRawPixmap);
   topCamRawLabel->setMinimumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   topCamRawLabel->setMaximumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   layout->addWidget(topCamRawLabel, 0, 0, 1, 2);

   botImageRawPixmap = QPixmap(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   botImageRawPixmap.fill(Qt::darkGray);
   botCamRawLabel  = new QLabel();
   botCamRawLabel->setPixmap(botImageRawPixmap);
   botCamRawLabel->setMinimumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   botCamRawLabel->setMaximumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   layout->addWidget(botCamRawLabel, 1, 0, 1, 2);

   topImageFoveaPixmap = QPixmap(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   topImageFoveaPixmap.fill(Qt::darkGray);
   topCamFoveaLabel  = new QLabel();
   topCamFoveaLabel->setPixmap(topImageFoveaPixmap);
   topCamFoveaLabel->setMinimumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   topCamFoveaLabel->setMaximumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   layout->addWidget(topCamFoveaLabel, 0, 2, 1, 2);

   botImageFoveaPixmap = QPixmap(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   botImageFoveaPixmap.fill(Qt::darkGray);
   botCamFoveaLabel  = new QLabel();
   botCamFoveaLabel->setPixmap(botImageFoveaPixmap);
   botCamFoveaLabel->setMinimumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   botCamFoveaLabel->setMaximumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   layout->addWidget(botCamFoveaLabel, 1, 2, 1, 2);

   topImageEdgePixmap = QPixmap(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   topImageEdgePixmap.fill(Qt::darkGray);
   topCamEdgeLabel  = new QLabel();
   topCamEdgeLabel->setPixmap(topImageEdgePixmap);
   topCamEdgeLabel->setMinimumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   topCamEdgeLabel->setMaximumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   layout->addWidget(topCamEdgeLabel, 0, 4, 1, 2);

   botImageEdgePixmap = QPixmap(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   botImageEdgePixmap.fill(Qt::darkGray);
   botCamEdgeLabel  = new QLabel();
   botCamEdgeLabel->setPixmap(botImageEdgePixmap);
   botCamEdgeLabel->setMinimumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   botCamEdgeLabel->setMaximumSize(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);
   layout->addWidget(botCamEdgeLabel, 1, 4, 1, 2);

   QVBoxLayout *vboxButtons = new QVBoxLayout ();

   /* Buttons */

   loadTopColour = new QPushButton("Load top");
   loadBotColour = new QPushButton("Load bot");
   resetColour = new QPushButton("Reset");
   saveTopColour = new QPushButton("Save top");
   saveBotColour = new QPushButton("Save bot");
   runColourTop = new QPushButton("Run (top)");
   runColourBot = new QPushButton("Run (bot)");
   runFillPointsGreen = new QPushButton("Fill classification (green)");
   runFillPointsWhite = new QPushButton("Fill classification (white)");

   saveTopColour->setEnabled(false);
   saveBotColour->setEnabled(false);
   runFillPointsGreen->setEnabled(false);
   runFillPointsWhite->setEnabled(false);

   /* Options for selecting gradient operator */

   QGroupBox *groupBoxGradient = new QGroupBox (tr ("Gradient Operator"));
   QVBoxLayout *vboxGradient = new QVBoxLayout ();

   radioNAIVE = new QRadioButton (tr ("Naive"));
   radioSOBEL = new QRadioButton (tr ("Sobel"));
   radioROBERTS = new QRadioButton (tr ("Robert's Cross"));
   radioISOTROPIC = new QRadioButton (tr ("Isotropic"));

   radioSOBEL->setChecked(true);

   vboxGradient->addWidget (radioNAIVE);
   vboxGradient->addWidget (radioSOBEL);
   vboxGradient->addWidget (radioROBERTS);
   vboxGradient->addWidget (radioISOTROPIC);

   groupBoxGradient->setLayout (vboxGradient);

   vboxButtons->addWidget(groupBoxGradient);

   // Y / U / V

   QGroupBox *groupBoxYUV = new QGroupBox (tr ("Gradient on Y/U/V"));
   QVBoxLayout *vboxYUV = new QVBoxLayout ();

   radioY = new QRadioButton (tr ("Y"));
   radioU = new QRadioButton (tr ("U"));
   radioV = new QRadioButton (tr ("V"));

   radioY->setChecked(true);

   vboxYUV->addWidget (radioY);
   vboxYUV->addWidget (radioU);
   vboxYUV->addWidget (radioV);

   groupBoxYUV->setLayout (vboxYUV);

   vboxButtons->addWidget(groupBoxYUV);

   stepSizeNum = new QLineEdit("1");
   vboxButtons->addWidget(stepSizeNum);

   // Thresholding?

   QGroupBox *groupBoxThreshold = new QGroupBox ();
   QVBoxLayout *vboxThreshold = new QVBoxLayout ();
   thresholdCheckbox = new QCheckBox("Threshold?");

   groupBoxThreshold->setLayout (vboxThreshold);

   thresholdNum = new QLineEdit("100");
   thresholdAutoTop = new QRadioButton(tr("Run Otsu (top)"));
   thresholdAutoBot = new QRadioButton(tr("Run Otsu (bot)"));
   thresholdNoAuto = new QRadioButton(tr("No auto threshold"));

   thresholdNoAuto->setChecked(true);

   vboxThreshold->addWidget(thresholdCheckbox);
   vboxThreshold->addWidget(thresholdNum);
   vboxThreshold->addWidget(thresholdAutoTop);
   vboxThreshold->addWidget(thresholdAutoBot);
   vboxThreshold->addWidget(thresholdNoAuto);

   vboxButtons->addWidget(groupBoxThreshold);
   // Other

   resultInfo = new QLabel();
   resultInfo->setFont(QFont("Monospace"));
   resultInfo->setText("Status");
   resultInfo->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);

   showDetectedEdgesCheckbox = new QCheckBox("Show detected edges?");

   vboxButtons->addWidget (loadTopColour);
   vboxButtons->addWidget (loadBotColour);
   vboxButtons->addWidget (resetColour);
   vboxButtons->addWidget (saveTopColour);
   vboxButtons->addWidget (saveBotColour);
   vboxButtons->addWidget (showDetectedEdgesCheckbox);
   vboxButtons->addWidget (runColourTop);
   vboxButtons->addWidget (runColourBot);
   vboxButtons->addWidget (runFillPointsGreen);
   vboxButtons->addWidget (runFillPointsWhite);
   vboxButtons->addWidget (resultInfo);

   layout->addLayout(vboxButtons, 0, 6, 2, 1, Qt::AlignTop | Qt::AlignLeft);

   connect(loadTopColour, SIGNAL(released()), this, SLOT(loadTopNnmc()));
   connect(loadBotColour, SIGNAL(released()), this, SLOT(loadBotNnmc()));
   connect(saveTopColour, SIGNAL(released()), this, SLOT(saveTopNnmc()));
   connect(saveBotColour, SIGNAL(released()), this, SLOT(saveBotNnmc()));
   connect(resetColour, SIGNAL(released()), this, SLOT(resetNnmc()));
   connect(runColourTop, SIGNAL(released()), this, SLOT(runNnmcTop()));
   connect(runColourBot, SIGNAL(released()), this, SLOT(runNnmcBot()));
   connect(runFillPointsGreen, SIGNAL(released()), this, SLOT(runNnmcFillPointsGreen()));
   connect(runFillPointsWhite, SIGNAL(released()), this, SLOT(runNnmcFillPointsWhite()));

   connect(radioNAIVE, SIGNAL(clicked(bool)), this, SLOT(safe_redraw()));
   connect(radioSOBEL, SIGNAL(clicked(bool)), this, SLOT(safe_redraw()));
   connect(radioROBERTS, SIGNAL(clicked(bool)), this, SLOT(safe_redraw()));
   connect(radioISOTROPIC, SIGNAL(clicked(bool)), this, SLOT(safe_redraw()));
   connect(radioY, SIGNAL(clicked(bool)), this, SLOT(safe_redraw()));
   connect(radioU, SIGNAL(clicked(bool)), this, SLOT(safe_redraw()));
   connect(radioV, SIGNAL(clicked(bool)), this, SLOT(safe_redraw()));
   connect(stepSizeNum, SIGNAL(textChanged(const QString &)), this, SLOT(safe_redraw()));
   connect(thresholdNum, SIGNAL(textChanged(const QString &)), this, SLOT(safe_redraw()));
   connect(thresholdCheckbox, SIGNAL(clicked(bool)), this, SLOT(safe_redraw()));
   connect(thresholdAutoTop, SIGNAL(clicked(bool)), this, SLOT(safe_redraw()));
   connect(thresholdAutoBot, SIGNAL(clicked(bool)), this, SLOT(safe_redraw()));
   connect(thresholdNoAuto, SIGNAL(clicked(bool)), this, SLOT(safe_redraw()));
}

void NNMCTab::safe_redraw() {
   if (haveData) {
      redraw();
   }
}

void NNMCTab::redraw() {
   if (topFrame || botFrame) {
      QImage *topImage;
      QImage *botImage;

      topImage = new QImage(TOP_IMAGE_COLS,
         TOP_IMAGE_ROWS,
         QImage::Format_RGB32);
      botImage = new QImage(BOT_IMAGE_COLS,
         BOT_IMAGE_ROWS,
         QImage::Format_RGB32);

      for (int row = 0; row < TOP_IMAGE_ROWS; row++) {
         for (int col = 0; col < TOP_IMAGE_COLS; col++) {
            topImage->setPixel(col, row, getRGB(col, row, topFrame, TOP_IMAGE_COLS));
         }
      }

      for (int row = 0; row < BOT_IMAGE_ROWS; row++) {
         for (int col = 0; col < BOT_IMAGE_COLS; col++) {
            botImage->setPixel(col, row, getRGB(col, row, botFrame, BOT_IMAGE_COLS));
         }
      }

      topImageRawPixmap = QPixmap::fromImage(
         topImage->scaled(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS));
      botImageRawPixmap = QPixmap::fromImage(
         botImage->scaled(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS));

      delete topImage;
      delete botImage;

      topCamRawLabel->setPixmap(topImageRawPixmap);
      botCamRawLabel->setPixmap(botImageRawPixmap);

      topImage = new QImage(TOP_IMAGE_COLS,
         TOP_IMAGE_ROWS,
         QImage::Format_RGB32);
      botImage = new QImage(BOT_IMAGE_COLS,
         BOT_IMAGE_ROWS,
         QImage::Format_RGB32);

      int buffer = stepSizeNum->text().toInt();

      Gradient_Operator gradient_operator;

      if (radioNAIVE->isChecked ()) {
         gradient_operator = NAIVE;
      } else if (radioSOBEL->isChecked ()) {
         gradient_operator = SOBEL;
      } else if (radioROBERTS->isChecked()) {
         gradient_operator = ROBERTS_CROSS;
      } else {
         gradient_operator = YUV_ISOTROPIC;
      }

      uint8_t (*getYUV)(const uint8_t*, int, int, int);

      if (radioY->isChecked()) {
         getYUV = &gety;
      } else if (radioU->isChecked()) {
         getYUV = &getu;
      } else {
         getYUV = &getv;
      }

      int threshold;

      if (thresholdNoAuto->isChecked()) {
         threshold = thresholdNum->text().toInt();
      } else {
         uint8_t (*getYUV)(const uint8_t*, int, int, int);

         if (radioY->isChecked()) {
            getYUV = &gety;
         } else if (radioU->isChecked()) {
            getYUV = &getu;
         } else {
            getYUV = &getv;
         }

         if (thresholdAutoTop->isChecked()) {
            threshold = green_yuv_classifier_top_->calculate_threshold(topFrame, TOP_IMAGE_ROWS, TOP_IMAGE_COLS, gradient_operator, getYUV);
         } else {
            threshold = green_yuv_classifier_bot_->calculate_threshold(botFrame, BOT_IMAGE_ROWS, BOT_IMAGE_COLS, gradient_operator, getYUV);
         }
         std::ostringstream s;
         s << threshold;

         thresholdNum->setText(QString(s.str().c_str()));
      }

      for (int row = buffer; row < TOP_IMAGE_ROWS - buffer; row++) {
         for (int col = buffer; col < TOP_IMAGE_COLS - buffer; col++) {
            topImage->setPixel(col, row, getEdge(col, row, topFrame, TOP_IMAGE_COLS, gradient_operator, getYUV, threshold));
         }
      }

      for (int row = buffer; row < BOT_IMAGE_ROWS - buffer; row++) {
         for (int col = buffer; col < BOT_IMAGE_COLS - buffer; col++) {
            botImage->setPixel(col, row, getEdge(col, row, botFrame, BOT_IMAGE_COLS, gradient_operator, getYUV, threshold));
         }
      }

      topImageEdgePixmap = QPixmap::fromImage(
         topImage->scaled(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS));
      botImageEdgePixmap = QPixmap::fromImage(
         botImage->scaled(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS));

      delete topImage;
      delete botImage;

      topCamEdgeLabel->setPixmap(topImageEdgePixmap);
      botCamEdgeLabel->setPixmap(botImageEdgePixmap);
   }

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

         // Scale images up to real size to draw overlays
      QPixmap t = QPixmap::fromImage(
       topImage->scaled(2*DEBUG_IMAGE_COLS, 2*DEBUG_IMAGE_ROWS));
      botImageFoveaPixmap = QPixmap::fromImage(
       botImage->scaled(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS));

      drawOverlays(&t, &botImageFoveaPixmap);

         // Rescale the top image back to 640x480 to fit the screen
      topImageFoveaPixmap = t.scaled(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS);

      delete topImage;
      delete botImage;
   }
   else {
      topImageFoveaPixmap = QPixmap(IMAGE_COLS, IMAGE_ROWS);
      topImageFoveaPixmap.fill(Qt::darkRed);
      botImageFoveaPixmap = QPixmap(IMAGE_COLS, IMAGE_ROWS);
      botImageFoveaPixmap.fill(Qt::darkRed);
   }
   topCamFoveaLabel->setPixmap(topImageFoveaPixmap);
   botCamFoveaLabel->setPixmap(botImageFoveaPixmap);
}

void NNMCTab::drawOverlays(QPixmap *topImage, QPixmap *botImage) {
   if (!blackboard) return;

   std::vector<FootInfo>           feet_boxes       = readFrom(vision, feet_boxes);
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


void NNMCTab::drawImage(QImage *topImage, QImage *botImage) {
   if (topFrame && botFrame) {
      VisionInfoIn info_in;
      vision->processFrame(CombinedFrame(topFrame, botFrame), info_in);

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
   }
   else {
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
void NNMCTab::newNaoData(NaoData *naoData) {
   if (!naoData || !naoData->getCurrentFrame().blackboard) {  // clean up display, as read is finished
      topImageRawPixmap.fill(Qt::darkGray);
      topCamRawLabel->setPixmap(topImageRawPixmap);
      botImageRawPixmap.fill(Qt::darkGray);
      botCamRawLabel->setPixmap(botImageRawPixmap);

      topImageFoveaPixmap.fill(Qt::darkGray);
      topCamFoveaLabel->setPixmap(topImageFoveaPixmap);
      botImageFoveaPixmap.fill(Qt::darkGray);
      botCamFoveaLabel->setPixmap(botImageFoveaPixmap);

      topImageEdgePixmap.fill(Qt::darkGray);
      topCamEdgeLabel->setPixmap(topImageEdgePixmap);
      botImageEdgePixmap.fill(Qt::darkGray);
      botCamEdgeLabel->setPixmap(botImageEdgePixmap);
   } else if (naoData->getFramesTotal() != 0) {
      haveData = true;
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
      }
   }
}

void NNMCTab::loadTopNnmc() {
   QString fileName = QFileDialog::getOpenFileName(this, "Load Top Nnmc File");

   if (fileName != "") {
      vision->loadNnmc(true, fileName.toStdString().c_str());

      nnmcLoaded = true;
      saveTopColour->setEnabled(true);
      runFillPointsGreen->setEnabled(true);
      runFillPointsWhite->setEnabled(true);
      redraw();
      resultInfo->setText("Loaded Top");
   }
}

void NNMCTab::loadBotNnmc() {
   QString fileName = QFileDialog::getOpenFileName(this, "Load Bot Nnmc File");

   if (fileName != "") {
      vision->loadNnmc(false, fileName.toStdString().c_str());

      nnmcLoaded = true;
      saveBotColour->setEnabled(true);
      runFillPointsGreen->setEnabled(true);
      runFillPointsWhite->setEnabled(true);
      redraw();
      resultInfo->setText("Loaded Bot");
   }
}

void NNMCTab::saveTopNnmc() {
   QString fileName = QFileDialog::getSaveFileName(this, "Save Top Nnmc File");
   if (fileName != "") {
      vision->saveNnmc(true, fileName.toStdString().c_str());
      resultInfo->setText("Saved Top");
   }
}

void NNMCTab::saveBotNnmc() {
   QString fileName = QFileDialog::getSaveFileName(this, "Save Bot Nnmc File");
   if (fileName != "") {
      vision->saveNnmc(false, fileName.toStdString().c_str());
      resultInfo->setText("Saved Bot");
   }
}

void NNMCTab::runNnmcTop() {
   saveTopColour->setEnabled(true);
   runFillPointsGreen->setEnabled(true);
   runFillPointsWhite->setEnabled(true);

   int stepsize = stepSizeNum->text().toInt();

   if (showDetectedEdgesCheckbox->isChecked()) {
      uint8_t noiseReducedTopFrame[TOP_IMAGE_ROWS * TOP_IMAGE_COLS * 2];
      noiseReduce(topFrame, noiseReducedTopFrame, TOP_IMAGE_ROWS, TOP_IMAGE_COLS);

      displayScanLineResults(topCamFoveaLabel, topImageFoveaPixmap, noiseReducedTopFrame, true, TOP_IMAGE_ROWS, TOP_IMAGE_COLS, stepsize);

      uint8_t noiseReducedBotFrame[BOT_IMAGE_ROWS * BOT_IMAGE_COLS * 2];
      noiseReduce(botFrame, noiseReducedBotFrame, BOT_IMAGE_ROWS, BOT_IMAGE_COLS);

      displayScanLineResults(botCamFoveaLabel, botImageFoveaPixmap, noiseReducedBotFrame, false, BOT_IMAGE_ROWS, BOT_IMAGE_COLS, stepsize);
   }
   else {
      uint8_t noiseReducedTopFrame[TOP_IMAGE_ROWS * TOP_IMAGE_COLS * 2];
      noiseReduce(topFrame, noiseReducedTopFrame, TOP_IMAGE_ROWS, TOP_IMAGE_COLS);

      vision->runNnmc(true, noiseReducedTopFrame, stepsize);
      redraw();
      resultInfo->setText("Top run");
   }
}

void NNMCTab::runNnmcBot() {
   saveBotColour->setEnabled(true);
   runFillPointsGreen->setEnabled(true);
   runFillPointsWhite->setEnabled(true);

   int stepsize = stepSizeNum->text().toInt();

   if (showDetectedEdgesCheckbox->isChecked()) {
      uint8_t noiseReducedTopFrame[TOP_IMAGE_ROWS * TOP_IMAGE_COLS * 2];
      noiseReduce(topFrame, noiseReducedTopFrame, TOP_IMAGE_ROWS, TOP_IMAGE_COLS);

      displayScanLineResults(topCamFoveaLabel, topImageFoveaPixmap, noiseReducedTopFrame, true, TOP_IMAGE_ROWS, TOP_IMAGE_COLS, stepsize);

      uint8_t noiseReducedBotFrame[BOT_IMAGE_ROWS * BOT_IMAGE_COLS * 2];
      noiseReduce(botFrame, noiseReducedBotFrame, BOT_IMAGE_ROWS, BOT_IMAGE_COLS);

      displayScanLineResults(botCamFoveaLabel, botImageFoveaPixmap, noiseReducedBotFrame, false, BOT_IMAGE_ROWS, BOT_IMAGE_COLS, stepsize);
   }
   else {
      uint8_t noiseReducedBotFrame[BOT_IMAGE_ROWS * BOT_IMAGE_COLS * 2];
      noiseReduce(botFrame, noiseReducedBotFrame, BOT_IMAGE_ROWS, BOT_IMAGE_COLS);

      vision->runNnmc(false, noiseReducedBotFrame, stepsize);
      redraw();
      resultInfo->setText("Bot run");
   }
}

void NNMCTab::runNnmcFillPointsGreen() {
   vision->runFillPoints(true, true);
   vision->runFillPoints(false, true);
   redraw();
   resultInfo->setText("Nnmc filled (green)");
}

void NNMCTab::runNnmcFillPointsWhite() {
   vision->runFillPoints(true, false);
   vision->runFillPoints(false, false);
   redraw();
   resultInfo->setText("Nnmc filled (white)");
}

void NNMCTab::resetNnmc() {
   vision->resetNnmc(true);
   vision->resetNnmc(false);
   redraw();
   resultInfo->setText("Reset");
}

QRgb NNMCTab::getEdge(unsigned int col,
                 unsigned int row,
                 const uint8_t *yuv,
                 int num_cols,
                 Gradient_Operator g,
                 uint8_t (*getYUV)(const uint8_t*, int, int, int),
                 int threshold) {

   int stepsize = stepSizeNum->text().toInt();

   // Function is the same for top and bottom. Doesn't matter which instance we use
   int gradient = green_yuv_classifier_top_->calculate_gradient(yuv, row, col, num_cols, g, getYUV, stepsize);
   gradient = std::min(gradient, 255);

   if (thresholdCheckbox->isChecked() && gradient > threshold) {
      return qRgb(255, 105, 180);
   }

   return qRgb(gradient, gradient, gradient);
}

void NNMCTab::displayScanLineResults(QLabel *qlabel,
      QPixmap qpixmap, const uint8_t *image, bool top, int n_rows, int n_cols, int stepsize) {

   QImage *q_image;

   q_image = new QImage(n_cols,
      n_rows,
      QImage::Format_RGB32);

   // For future samplings (of the bottom frame), we should use isGreen / isWhite to determine what our starting pixel is
   // for cases where the line we are standing on or our shoulder is in view

   int inc = 1;

   // These should be based on what kernel we use for our gradient calculation

   //int x_buffer = 1;
   //int y_buffer = 1;
   int x_buffer = stepsize;
   int y_buffer = stepsize;

   // We assume for the first sample that the first pixel is green

   // We flip between Green and white until we observe at least 1 transition
   // This needs to change for alternate head_yaw angles since sometimes there isn't any transition
   // Shouldn't be much of a problem since this logic is fitted for the initial head on sample

   int start_y;
   int num_transitions;
   bool detectingGreen;
   int transition_threshold = 3;
   bool noEdge = false;

   int num_preceding_consecutive_edges = 0;
   bool isEdge;
   // Green -> White -> Green -> White / Background

   Gradient_Operator grad_op;

   if (radioNAIVE->isChecked ()) {
      grad_op = NAIVE;
   } else if (radioSOBEL->isChecked ()) {
      grad_op = SOBEL;
   } else if (radioROBERTS->isChecked()) {
      grad_op = ROBERTS_CROSS;
   } else {
      grad_op = YUV_ISOTROPIC;
   }

   int threshold_buffer = 20;

   // Function is the same for top and bottom. Doesn't matter which instance we use
   int threshold_otsu = green_yuv_classifier_top_->calculate_threshold(image, n_rows, n_cols, grad_op, &gety);
   int threshold_y = threshold_otsu;
    //int threshold_u = calculate_threshold(image, n_rows, n_cols, grad_op, &gety);
    //int threshold_v = calculate_threshold(image, n_rows, n_cols, grad_op, &gety);

   noEdge = threshold_y < 30;

   for (int x_pos = x_buffer; x_pos < n_cols - x_buffer; x_pos += inc) {
      detectingGreen = true;
      num_transitions = 0;
      num_preceding_consecutive_edges = 0;
        // Start from the bottom of the image

        /*
        if (top) {
            start_y = info_in.cameraToRR.getTopEndScanCoord(x_pos) - 1;
        }
        else {
            start_y = info_in.cameraToRR.getBotEndScanCoord(x_pos) - 1;
        }
        */

      start_y = n_rows - 1;

      int firstWhiteY = 0;
      int lastGreenY = 0;

      for (int y_pos = start_y - y_buffer; y_pos >= y_buffer; y_pos -= inc) {
         // Make threshold more strict the further up you are in the image
         if (top) {
            if (y_pos > 6 * start_y / 10) {
               threshold_y = threshold_otsu;
            }
            else if (y_pos > 5 * start_y / 10) {
               threshold_y = std::max(0, threshold_otsu - 30);
            }
            else if (y_pos > 4 * start_y / 10) {
               threshold_y = std::max(0, threshold_otsu - 60);
            }
            else {
               threshold_y = std::max(0, threshold_otsu - 90);
            }
         }
         else {
            threshold_y = threshold_otsu;
         }

         if (noEdge) {
           //int y_val = gety(image, y_pos, x_pos, n_cols);
           //int u_val = getu(image, y_pos, x_pos, n_cols);
           //int v_val = getv(image, y_pos, x_pos, n_cols);

           q_image->setPixel(x_pos, y_pos, QColor("green").rgb());
           //is_green_[y_val >> Y_BITSHIFT][u_val >> U_BITSHIFT][v_val >> V_BITSHIFT] = true;
        }
        else {
            // Function is the same for top and bottom. Doesn't matter which instance we use
            int grad_y = green_yuv_classifier_top_->calculate_gradient(image, y_pos, x_pos, n_cols, grad_op, &gety, stepsize);
            //int grad_u = green_yuv_classifier->calculate_gradient(image, y_pos, x_pos, n_cols, grad_op, &getu, stepsize);
            //int grad_v = green_yuv_classifier->calculate_gradient(image, y_pos, x_pos, n_cols, grad_op, &getv, stepsize);

            bool exceeded_y = grad_y > threshold_y;
            //bool exceeded_u = grad_u > threshold.u_;
            //bool exceeded_v = grad_v > threshold.v_;

            bool exceeded_thresh = exceeded_y;
            int y_val = gety(image, y_pos, x_pos, n_cols);
            //int u_val = getu(image, y_pos, x_pos, n_cols);
            //int v_val = getv(image, y_pos, x_pos, n_cols);

                /*
                    Proposed algorithm
                    Assume Green->White is ok
                    Store the y value corresponding to the first detected "white" pixel
                    When we see subsequent edges in white, check if the first value after the edge is < this
                    If it is, then assume we have transitioned back to green

                */

            if (exceeded_thresh) {
               isEdge = true;
               num_preceding_consecutive_edges++;
            }
            else {
               isEdge = false;

               if (num_preceding_consecutive_edges != 0) {
                  num_transitions++;
                  if (detectingGreen) {
                     if (y_val > lastGreenY + threshold_buffer) {
                        // Begin detecting white
                        detectingGreen = false;
                        firstWhiteY = y_val;
                        //num_transitions++;
                     }
                  }
                  else {
                     if (y_val < firstWhiteY - threshold_buffer) {
                        detectingGreen = true;
                        //num_transitions++;
                     }
                  }

                  if (num_transitions > transition_threshold) {
                     break;
                  }

                  num_preceding_consecutive_edges = 0;
               }

                /*
                    Some problems
                    Edge may extend for multiple pixels

                    We only blindly detect the first transition
                        Subsequent ones are informed by our current classification
                 */

                /*
                if (exceeded_thresh) {
                    isEdge = true;

                    if (num_transitions == 0) {
                        // We only blindly detect the first transition
                        detectingGreen = !detectingGreen;
                        num_transitions++;
                    }
                    else {
                        // Subsequent ones are informed by our current classification
                        // If we are detecting one colour, reach an edge and it is another colour, then switch
                        if (num_preceding_consecutive_edges == 0) {
                            if (detectingGreen && isWhite(y_val, u_val, v_val)) {
                                detectingGreen = false;
                                num_transitions++;
                            }
                            else if (!detectingGreen && isGreen(y_val, u_val, v_val, true)) {
                                detectingGreen = true;
                                num_transitions++;
                            }
                            if (num_transitions > transition_threshold) {
                                break;
                            }
                        }
                    }
                    num_preceding_consecutive_edges++;
                }
                else {
                    isEdge = false;
                    num_preceding_consecutive_edges = 0;
                }
                */

               if (!isEdge) {
                  if (detectingGreen) {
                     q_image->setPixel(x_pos, y_pos, QColor("green").rgb());
                     //is_green_[y_val >> Y_BITSHIFT][u_val >> U_BITSHIFT][v_val >> V_BITSHIFT] = true;
                     lastGreenY = y_val;
                  }
                  else {
                     q_image->setPixel(x_pos, y_pos, QColor("white").rgb());
                     //is_white_[y_val >> Y_BITSHIFT][u_val >> U_BITSHIFT][v_val >> V_BITSHIFT] = true;
                  }
               }
               else {
                  q_image->setPixel(x_pos, y_pos, QColor("red").rgb());
               }
            }
         }
      }
   }

   qpixmap = QPixmap::fromImage(
      q_image->scaled(DEBUG_IMAGE_COLS, DEBUG_IMAGE_ROWS));

   delete q_image;
   qlabel->setPixmap(qpixmap);
}

void NNMCTab::noiseReduce(const uint8_t *frame, uint8_t *noiseReducedFrame, int n_rows, int n_cols) {
   int neighbourhood = 3;

   for (int i = 0; i < n_rows; i++) {
      for (int j = 0; j < n_cols; j++) {
         int linIndex = 2 * (i * n_cols + j / 2 * 2);
         (const_cast<uint8_t *>(noiseReducedFrame) + (linIndex))[1] =
            (const_cast<uint8_t *>(frame) + (2 * (i * n_cols + j / 2 * 2)))[1];

         (const_cast<uint8_t *>(noiseReducedFrame) + (linIndex))[3] =
            (const_cast<uint8_t *>(frame) + (linIndex))[3];

         if (i >= neighbourhood && i < n_rows - neighbourhood && j >= neighbourhood && j < n_cols - neighbourhood) {
            int yIndex = 2 * (j & 1);

            std::vector <uint8_t> y_vals;

            for (int ii = i - neighbourhood; ii <= i + neighbourhood; ii++) {
               for (int jj = j - neighbourhood; jj <= j + neighbourhood; jj++) {
                  y_vals.push_back((const_cast<uint8_t *>(frame) + (2 * (ii * n_cols + jj / 2 * 2)))[2 * (jj & 1)]);
               }
            }

            sort(y_vals.begin(), y_vals.end());

            (const_cast<uint8_t *>(noiseReducedFrame) + (linIndex))[yIndex] =
               y_vals[((2 * neighbourhood + 1) * (2 * neighbourhood + 1)) / 2];
         }
         else {
            (const_cast<uint8_t *>(noiseReducedFrame) + (linIndex))[2 * (j & 1)] =
               (const_cast<uint8_t *>(frame) + (linIndex))[2 * (j & 1)];
         }
      }
   }
}

/*
uint8_t* getpixelpair(const uint8_t* yuv, int row, int col, int num_cols) {
   return const_cast<uint8_t *>(yuv)
      + (row * num_cols + col / 2 * 2) * 2;
}

uint8_t gety(const uint8_t* yuv, int row, int col, int num_cols) {
   return getpixelpair(yuv, row, col, num_cols)[2 * (col & 1)];
}

uint8_t getu(const uint8_t* yuv, int row, int col, int num_cols) {
   return getpixelpair(yuv, row, col, num_cols)[1];
}

uint8_t getv(const uint8_t* yuv, int row, int col, int num_cols) {
   return getpixelpair(yuv, row, col, num_cols)[3];
}*/

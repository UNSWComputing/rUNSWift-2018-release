#include <QMenu>
#include <QMenuBar>
#include <QFileDialog>
#include <QDebug>
#include <QPainter>
#include <QMouseEvent>
#include <vector>
#include <iostream>
#include <utility>
#include <sstream>
#include <string>
#include "visionTab.hpp"
#include "perception/vision/Vision.hpp"
#include "perception/vision/other/YUV.hpp"
#include "blackboard/Blackboard.hpp"
#include "utils/OverlayPainter.hpp"
#include "utils/CPlaneColours.hpp"

#include "perception/vision/Region.hpp"

using namespace std;


std::string displayPoint2(Point p) {
	std::stringstream ss;

	ss << "[" << p.x() << "," << p.y() << "]";
	return ss.str();
}

std::string displayBBox(BBox b) {
	std::stringstream ss;
	ss << displayPoint2(b.a) << " to " << displayPoint2(b.b);
	return ss.str();
}

void drawBoundingBox(OverlayPainter &painter, BBox box, QColor colour, const int density, const int rowOffset) {
	QPainterPath boxPath;
	boxPath.moveTo(box.a.x() * density, box.a.y() * density + rowOffset);
	boxPath.lineTo(box.b.x() * density, box.a.y() * density + rowOffset);
	boxPath.lineTo(box.b.x() * density, box.b.y() * density + rowOffset);
	boxPath.lineTo(box.a.x() * density, box.b.y() * density + rowOffset);
	boxPath.lineTo(box.a.x() * density, box.a.y() * density + rowOffset);
	painter.drawLinePath(boxPath, colour);
}

template<class T>
string myts(const T& t) {
   stringstream ss;
   ss << t;
   return ss.str();
}

string operator+(string a, string b) {
   stringstream ss;
   ss << a << b;
   return ss.str().c_str();
}

VisionTab::VisionTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision)
   : zoomLevel(1), zoomLog(0), mouseX(0), mouseY(0),
     prevMouseX(0), prevMouseY(0), prevZoomLevel(1),
     nnmc_loaded(false)
{
   this->parent = parent;
   this->vision = vision;

   initMenu(menuBar);
   init();
   currentFrame = NULL;
   topFrame = NULL;
   botFrame = NULL;
}


void VisionTab::initMenu(QMenuBar *menuBar) {
   visionMenu = new QMenu("Vision");
   menuBar->addMenu(visionMenu);

      // TODO: REFACTOR17
   reloadBothNnmcAct = new QAction(tr("Reload both NNMCs"), visionMenu);
   loadBotNnmcAct = new QAction(tr("Load Bottom NNMC"), visionMenu);
   loadTopNnmcAct = new QAction(tr("Load Top NNMC"   ), visionMenu);
   loadVisualWordsAct = new QAction(tr("Load Visual Words"      ), visionMenu);
   loadGoalMapAct = new QAction(tr("Load Goal Background Map"   ), visionMenu);
   loadAllAct = new QAction(tr("Load All Default Files"   ), visionMenu);

   // Need visual words before a goal map
   loadGoalMapAct->setEnabled(false);

   visionMenu->addAction(reloadBothNnmcAct);
   visionMenu->addAction(loadBotNnmcAct);
   visionMenu->addAction(loadTopNnmcAct);
   visionMenu->addAction(loadVisualWordsAct);
   visionMenu->addAction(loadGoalMapAct);
   visionMenu->addAction(loadAllAct);

   // connect the actions
   connect(reloadBothNnmcAct, SIGNAL(triggered()), this, SLOT(reloadBothNnmc()));
   connect(loadBotNnmcAct, SIGNAL(triggered()), this, SLOT(loadBotNnmc ()));
   connect(loadTopNnmcAct, SIGNAL(triggered()), this, SLOT(loadTopNnmc ()));
   connect(loadVisualWordsAct, SIGNAL(triggered()), this, SLOT(loadVisualWords ()));
   connect(loadGoalMapAct, SIGNAL(triggered()), this, SLOT(loadGoalMap ()));
   connect(loadAllAct, SIGNAL(triggered()), this, SLOT(loadAll ()));

}

void VisionTab::init() {
   layout = new QGridLayout();
   this->setLayout(layout);

   imagePixmap = QPixmap(IMAGE_COLS, IMAGE_ROWS);
   imagePixmap.fill(Qt::darkGray);
   camLabel  = new QLabel();
   camLabel->setPixmap(imagePixmap);

   layout->addWidget(camLabel, 0, 0, 1, 1);

   QGroupBox *groupBoxImage       = new QGroupBox (tr ("Image Type"));
   QGroupBox *groupBoxLowFeatures = new QGroupBox (tr ("Low Features"));
   QGroupBox *groupBoxFeatures    = new QGroupBox (tr ("Features"));

   groupBoxImage->setMaximumWidth    (200);
   groupBoxFeatures->setMaximumWidth (200);

   QVBoxLayout *vboxImage       = new QVBoxLayout ();
   QVBoxLayout *vboxLowFeatures = new QVBoxLayout ();
   QVBoxLayout *vboxFeatures    = new QVBoxLayout ();

   QVBoxLayout *vboxRight = new QVBoxLayout ();

   /* Initialise image radio buttons */
   radioImage        = new QRadioButton (tr ("Full Image"));
   radioSaliency     = new QRadioButton (tr ("Saliency Image"));
//    radioSaliencyEdge = new QRadioButton (tr ("Edge Saliency Image"));
//    radioSaliencyGrey = new QRadioButton (tr ("Grey Saliency Image"));



   //radioImage->setChecked (true);
   radioImage->setChecked(true);

   vboxImage->addWidget (radioImage);
   vboxImage->addWidget (radioSaliency);
//    vboxImage->addWidget (radioSaliencyEdge);
//    vboxImage->addWidget (radioSaliencyGrey);
//    vboxImage->addWidget (radioDogsFilter);

   groupBoxImage->setLayout (vboxImage);

   /* Initialise low feature check boxes */
   checkBottomImage      = new QCheckBox("BottomCamera", this);
   checkEdgeDirections   = new QCheckBox("Edge Directions", this);
   checkEdgeOrientations = new QCheckBox("Edge Orientations", this);
   checkPatches          = new QCheckBox("Patches", this);
   checkLowLines         = new QCheckBox("Low Lines", this);

   vboxLowFeatures->addWidget (checkBottomImage     );
   vboxLowFeatures->addWidget (checkEdgeDirections  );
   vboxLowFeatures->addWidget (checkEdgeOrientations);
   vboxLowFeatures->addWidget (checkPatches         );
   vboxLowFeatures->addWidget (checkLowLines        );

   groupBoxLowFeatures->setLayout (vboxLowFeatures);

   checkHorizon       = new QCheckBox("Horizon"       , this);
   checkBall          = new QCheckBox("Ball"          , this);
   checkFeet          = new QCheckBox("Feet"          , this);
   checkGoals         = new QCheckBox("Goals"         , this);
   checkRobots        = new QCheckBox("Robots"        , this);
   checkRobotsDebug   = new QCheckBox("Robots Debug"  , this);
   checkFieldEdges    = new QCheckBox("Field Edges"   , this);
   checkFieldFeatures = new QCheckBox("Field Features", this);
   checkFeetDebug     = new QCheckBox("Feet Debug"    , this);


   checkHorizon->setChecked(false);
   checkBall->setChecked(true);
   checkFeet->setChecked(false);
   checkGoals->setChecked(false);
   checkRobots->setChecked(false);
   checkRobotsDebug->setChecked(false);
   checkFieldEdges->setChecked(false);
   checkFieldFeatures->setChecked(false);

   vboxFeatures->addWidget(checkHorizon);
   vboxFeatures->addWidget(checkBall);
   vboxFeatures->addWidget(checkGoals);
   vboxFeatures->addWidget(checkRobots);
   vboxFeatures->addWidget(checkRobotsDebug);
   vboxFeatures->addWidget(checkFieldEdges);
   vboxFeatures->addWidget(checkFieldFeatures);
   vboxFeatures->addWidget(checkFeet);
   vboxFeatures->addWidget(checkFeetDebug);

   groupBoxFeatures->setLayout (vboxFeatures);

   /* Stitch together the right side */
   vboxRight->addWidget (groupBoxImage);
   vboxRight->addWidget (groupBoxLowFeatures);
   vboxRight->addWidget (groupBoxFeatures);

   layout->addLayout(vboxRight, 0, 2, 1, 1, Qt::AlignTop | Qt::AlignLeft);

   connect(radioImage, SIGNAL(clicked(bool)),
                    this, SLOT(redrawSlot()));
   connect(radioSaliency, SIGNAL(clicked(bool)),
                    this, SLOT(redrawSlot()));
//    connect(radioSaliencyEdge, SIGNAL(clicked(bool)),
//                     this, SLOT(redrawSlot()));
//    connect(radioSaliencyGrey, SIGNAL(clicked(bool)),
//                     this, SLOT(redrawSlot()));
   connect(checkBottomImage, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkEdgeDirections, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkEdgeOrientations, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkPatches, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkLowLines, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));

   connect(checkBall, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkFeet, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkFeetDebug, SIGNAL(stateChanged(int)),
                       this, SLOT(redrawSlot()));
   connect(checkGoals, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkRobots, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkRobotsDebug, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkFieldEdges, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkFieldFeatures, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));

   this->setMouseTracking(true);
   camLabel->setMouseTracking(true);
   camLabel->setAlignment(Qt::AlignTop);
   camLabel->setMinimumSize(IMAGE_COLS, IMAGE_ROWS);
   camLabel->setMaximumSize(IMAGE_COLS, IMAGE_ROWS);
   camLabel->installEventFilter(this);
}

void VisionTab::mouseMoveEvent(QMouseEvent * event) {
   mousePosition = event->pos();
   mousePosition -= camLabel->pos();
   if (currentFrame && mousePosition.x() >= 0 &&
                    mousePosition.x() < IMAGE_COLS &&
         mousePosition.y() >= 0 && mousePosition.y() < IMAGE_ROWS) {
      stringstream message;

      message << "(" << mousePosition.x() << "," << mousePosition.y() << ")";
      int cols = (!checkBottomImage->isChecked()) ?
                  TOP_IMAGE_COLS : BOT_IMAGE_COLS;
      QColor colour = QColor(this->getRGB(mousePosition.x(),
                      mousePosition.y(), currentFrame, cols));

      message << " - (" << colour.red() << ", " <<
              colour.green() << ", " << colour.blue() << ")";
      emit showMessage(QString(message.str().c_str()), 0);
   }
}

void VisionTab::redraw() {
   if (topFrame) {

      VisionInfoOut info_out = vision->processFrame(CombinedFrame(topFrame, botFrame), info_in);

      bool top = !(checkBottomImage->isChecked());

      lastRendering =  QImage(BOT_IMAGE_COLS, BOT_IMAGE_ROWS, QImage::Format_RGB32);
      if (top) {
         lastRendering =  QImage(IMAGE_COLS, IMAGE_ROWS, QImage::Format_RGB32);
      }
      if (radioImage->isChecked ()) {
         drawImage (&lastRendering, top);
      } else if (radioSaliency->isChecked ()) {
         drawSaliency (&lastRendering, top);
      } 
    //   else if (radioSaliencyEdge->isChecked ()) {
    //      drawSaliencyEdge (&lastRendering, top);
    //   } else if (radioSaliencyGrey->isChecked ()) {
    //      drawSaliencyGrey (&lastRendering, top);
    //   }

      if (checkEdgeDirections->isChecked ()) {
         drawEdgeDirections (&lastRendering, top);
      }

      drawOverlays(&lastRendering, info_out, top);
      imagePixmap = QPixmap(QPixmap::fromImage(
                    lastRendering.scaled(BOT_IMAGE_COLS, BOT_IMAGE_ROWS)));

      // Extra stuff for zoom
      // Only do it in the top camera
      if (top) {
         QTransform prevTransform;
         prevTransform = prevTransform.translate(+prevMouseX, +prevMouseY);
         prevTransform = prevTransform.scale(1.0/prevZoomLevel,
                                             1.0/prevZoomLevel);
         prevTransform = prevTransform.translate(-prevMouseX, -prevMouseY);

         QRectF realMouseCoords = prevTransform.mapRect(
         QRectF(mouseX, mouseY, 0, 0));
         mouseX = realMouseCoords.left();
         mouseY = realMouseCoords.top();

         QTransform transform;
         transform = transform.translate(+mouseX, +mouseY);
         transform = transform.scale(1.0/zoomLevel, 1.0/zoomLevel);
         transform = transform.translate(-mouseX, -mouseY);
         imagePixmap = QPixmap(QPixmap::fromImage(
            lastRendering.scaled(IMAGE_COLS, IMAGE_ROWS)));
         QPainter painter(&lastRendering);
         QRectF bound = transform.mapRect(QRectF(0, 0, IMAGE_COLS, IMAGE_ROWS));

         painter.drawImage(QRectF(0, 0, IMAGE_COLS, IMAGE_ROWS),
                           imagePixmap.toImage(), bound);

         prevZoomLevel = zoomLevel;
         prevMouseX = mouseX;
         prevMouseY = mouseY;
      }

   } else {
      imagePixmap = QPixmap(IMAGE_COLS, IMAGE_ROWS);
      imagePixmap.fill(Qt::darkGray);
   }

   imagePixmap = QPixmap::fromImage(lastRendering.scaled(BOT_IMAGE_COLS,
                                                         BOT_IMAGE_ROWS));
   camLabel->setPixmap(imagePixmap);
}



void VisionTab::drawImage(QImage *image, bool top) {
   int R = (top) ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;
   int C = (top) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
   for (int row = 0; row < R; ++row) {
      for (int col = 0; col < C; ++col) {
         if (top) {
            image->setPixel(col, row, getRGB(col, row, topFrame, C));
         } else {
            image->setPixel(col, row, getRGB(col, row, botFrame, C));
         }
      }
   }
}

void VisionTab::drawSaliency(QImage *image, bool top)
{
    if (top) {
        drawSaliencyRegion(image, vision->getFullRegionTop());
    } else {
        drawSaliencyRegion(image, vision->getFullRegionBot());
    }

   // TODO: Re-add REFACTOR17

   /*if (checkBall->isChecked()) {
      std::vector<boost::shared_ptr<FoveaT<hNone, eNone> > >
         &tfs = vision->ballDetection.trackingFoveas;
      std::vector<boost::shared_ptr<FoveaT<hNone, eNone> > >::iterator tf;

      for (tf = tfs.begin(); tf != tfs.end(); ++ tf) {
         if ((top && (*tf)->asFovea().top) || (!top && !(*tf)->asFovea().top)){
            drawSaliencyFovea(image, (*tf)->asFovea());
         }
      }

      std::vector<boost::shared_ptr<FoveaT<hNone, eGrey> > >
         &bfs = vision->ballDetection.ballFoveas;
      std::vector<boost::shared_ptr<FoveaT<hNone, eGrey> > >::iterator bf;

      for (bf = bfs.begin(); bf != bfs.end(); ++ bf) {
         if ((top && (*bf)->asFovea().top) || (!top && !(*bf)->asFovea().top)){
            drawSaliencyFovea(image, (*bf)->asFovea());
         }
      }
   }

   if (checkFieldFeatures->isChecked()) {
      std::vector<boost::shared_ptr<FoveaT<hNone, eGrey> > >
         &tfs = vision->fieldLineDetection.foveas;
      std::vector<boost::shared_ptr<FoveaT<hNone, eGrey> > >::iterator tf;

      for (tf = tfs.begin(); tf != tfs.end(); ++ tf) {
         if ((top && (*tf)->asFovea().top) || (!top && !(*tf)->asFovea().top)){
            drawSaliencyFovea(image, (*tf)->asFovea());
         }
      }
   }
   if (checkGoals->isChecked()) {
      std::vector<boost::shared_ptr<FoveaT<hGoals, eGrey> > >
         &gfs = vision->goalDetection.goalFoveas;
      std::vector<boost::shared_ptr<FoveaT<hGoals, eGrey> > >::iterator gf;

      for (gf = gfs.begin(); gf != gfs.end(); ++ gf) {
         if ((top && (*gf)->asFovea().top) ||
             (!top && !(*gf)->asFovea().top)){
            drawSaliencyFovea(image, (*gf)->asFovea());
         }
      }
   }*/
}

void VisionTab::drawSaliencyRegion(QImage *image, const RegionI& region)
{
   // TODO: Re-add REFACTOR17

    QPainter painter(image);
    for (RegionI::iterator_fovea it = region.begin_fovea(); it != region.end_fovea(); ++it) {
        QColor c = CPLANE_COLOURS[it.colour()];
        painter.fillRect(it.xAbs() * region.getDensity(), it.yAbs() * region.getDensity(), region.getDensity(), region.getDensity(), c);
    }
   /*BBox BB (fovea.bb.a * fovea.density, fovea.bb.b * fovea.density);

   int s_row, s_col;
   for (int row = BB.a.y(); row < BB.b.y(); ++ row) {
      for (int col = BB.a.x(); col < BB.b.x(); ++ col) {
         s_col = col / fovea.density - fovea.bb.a.x();
         s_row = row / fovea.density - fovea.bb.a.y();
         QColor c = CPLANE_COLOURS[fovea.colour(s_col, s_row)];
         image->setPixel(col, row, c.rgb ());
      }
   }
   QPainter painter(image);
   painter.setPen("black");
   painter.drawRect(BB.a.x(), BB.a.y(), BB.width(), BB.height());*/
}

// void VisionTab::drawSaliencyGrey (QImage *image, bool top)
// {
//    // TODO: Re-add REFACTOR17
//     const RegionI& region = (top) ? vision->getFullRegionTop() : vision->getFullRegionBot();

//     QColor c;
//     QPainter painter(image);

//     for (RegionI::iterator_fovea it = region.begin_fovea(); it != region.end_fovea(); ++it) {
//         int grey = it.grey() / 16;
//         c.setRgb(grey, grey, grey);
//         painter.fillRect(it.xAbs() * region.getDensity(), it.yAbs() * region.getDensity(), region.getDensity(), region.getDensity(), c);
//     }
//    /*int s_row, s_col;
//    int density = (top) ? TOP_SALIENCY_DENSITY : BOT_SALIENCY_DENSITY;
//    int R = (top) ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;
//    int C = (top) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
//    for (int row = 0; row < R; ++row) {
//       for (int col = 0; col < C; ++col) {
//          s_row = row / density;
//          s_col = col / density;
//          int grey = vision->topSaliency.asFovea().grey(s_col, s_row) / 16;
//          if (!top) {
//             grey = vision->botSaliency.asFovea().grey(s_col, s_row) / 16;
//          }
//          image->setPixel(col, row, grey << 16 | grey << 8 | grey);
//       }

//    }
//    return;
//    // Draw the high quality fovea if we have one
//    if (checkBall->isChecked()) {
//       int index = top ? 0 : 1;
//       if (vision->ballDetection.ballFoveas.size() > 0) {
//          //drawSaliencyEdgeFovea(image, vision->ballDetection.ballFoveas[index]->asFovea());
//          const Fovea fovea = vision->ballDetection.ballFoveas[index]->asFovea();
//          density = fovea.density;
//          R = fovea.bb.width()*fovea.density;
//          C = fovea.bb.height() * fovea.density;
//          for (int row = 0; row < R; ++row) {
//             for (int col = 0; col < C; ++col) {
//                s_row = row / density;
//                s_col = col / density;
//                int grey = fovea.grey(s_col, s_row) / 16;
//                if (!top) {
//                   grey = fovea.grey(s_col, s_row) / 16;
//                }
//                image->setPixel(col, row, grey << 16 | grey << 8 | grey);
//             }
//          }
//       }
//    }*/
// }


// void VisionTab::drawSaliencyEdge (QImage *image, bool top)
// {
//    // TODO: Re-add REFACTOR17
//     if (top) {
//         drawSaliencyEdgeRegion(image, vision->getFullRegionTop());
//     } else {
//         drawSaliencyEdgeRegion(image, vision->getFullRegionBot());
//     }
//    /*if (top) {
//       drawSaliencyEdgeFovea(image, vision->topSaliency.asFovea());
//    } else {
//       drawSaliencyEdgeFovea(image, vision->botSaliency.asFovea());
//    }

//    if (checkBall->isChecked()) {
//       std::vector<boost::shared_ptr<FoveaT<hNone, eGrey> > >
//          &bfs = vision->ballDetection.ballFoveas;
//       std::vector<boost::shared_ptr<FoveaT<hNone, eGrey> > >::iterator bf;

//       for (bf = bfs.begin(); bf != bfs.end(); ++ bf) {
//          if ((top && (*bf)->asFovea().top) || (!top && !(*bf)->asFovea().top)){
//             drawSaliencyEdgeFovea(image, (*bf)->asFovea());
//          }
//       }
//    }

//    if (checkFieldFeatures->isChecked()) {
//       std::vector<boost::shared_ptr<FoveaT<hNone, eGrey> > >
//          &tfs = vision->fieldLineDetection.foveas;
//       std::vector<boost::shared_ptr<FoveaT<hNone, eGrey> > >::iterator tf;

//       for (tf = tfs.begin(); tf != tfs.end(); ++ tf) {
//          drawSaliencyEdgeFovea(image, (*tf)->asFovea());
//       }
//    }
//    if (checkGoals->isChecked()) {
//       std::vector<boost::shared_ptr<FoveaT<hGoals, eGrey> > >
//          &gfs = vision->goalDetection.goalFoveas;
//       std::vector<boost::shared_ptr<FoveaT<hGoals, eGrey> > >::iterator gf;

//       for (gf = gfs.begin(); gf != gfs.end(); ++ gf) {
//          if ((top && (*gf)->asFovea().top) ||
//              (!top && !(*gf)->asFovea().top)){
//             drawSaliencyEdgeFovea(image, (*gf)->asFovea());
//          }
//       }
//    }*/
// }

// void VisionTab::drawSaliencyEdgeRegion(QImage *image, const RegionI& region)
// {
//    // TODO: Re-add REFACTOR17
//     // int total_mag = 16 * (((fovea.edge_weights >>  0) & 0xFF) +
//     //                     ((fovea.edge_weights >>  8) & 0xFF) +
//     //                     ((fovea.edge_weights >> 16) & 0xFF));
//     int total_mag = 16;

//     int dx, dy;
//     double mag, theta;
//     int h, s, v;

//     QColor c;
//     QPoint p1, p2;

//     QPainter painter(image);
//     for (RegionI::iterator_fovea it = region.begin_fovea(); it != region.end_fovea(); ++it) {
//         const Point &e = it.edge();

//         dx = e[0];
//         dy = e[1];

//         theta = atan2 (dy, dx);
//         if (theta < 0) {
//             theta = 2 * M_PI + theta;
//         }

//         mag = 4 * hypot (dy, dx) / total_mag;

//         h = (theta * 180) / (2 * M_PI);
//         s = 255;
//         v = mag < 255 ? mag : 255;

//         c.setHsv(h, s, v);
//         painter.fillRect(it.xAbs() * region.getDensity(), it.yAbs() * region.getDensity(), region.getDensity(), region.getDensity(), c);

//     }
//    /*BBox BB(fovea.bb.a * fovea.density, fovea.bb.b * fovea.density);


//    QPainter painter (image);

//    int rect_size = fovea.density;

//    int total_mag = 16 * (((fovea.edge_weights >>  0) & 0xFF) +
//                          ((fovea.edge_weights >>  8) & 0xFF) +
//                          ((fovea.edge_weights >> 16) & 0xFF));
//    int dx, dy;
//    double mag, theta;
//    int h, s, v;

//    QColor c;
//    QPoint p1, p2;

//    for (int row = BB.a.y(); row < BB.b.y(); row += fovea.density) {
//       for (int col = BB.a.x(); col < BB.b.x(); col += fovea.density) {
//          const Point &e = fovea.edge(col / fovea.density - fovea.bb.a.x(),
//                                      row / fovea.density - fovea.bb.a.y());

//          dx = e[0];
//          dy = e[1];

//          theta = atan2 (dy, dx);
//          if (theta < 0) {
//             theta = 2 * M_PI + theta;
//          }

//          mag = 4 * hypot (dy, dx) / total_mag;

//          h = (theta * 180) / (2 * M_PI);
//          //h = 0;
//          s = 255;
//          v = mag < 255 ? mag : 255;
//          //v = mag < 50 ? 0 : 255;

//          c.setHsv (h, s, v);
//          painter.fillRect (QRect (col, row, rect_size, rect_size), c.toRgb ());

//          //if (abs(dy) < 250 && abs(dx) < 250 && abs(dy) > 30 && abs(dx) > 30)image->setPixel(col, row, 255 << 16 | 255 << 8 | 255);
//       }
//    }

//    painter.setPen("white");
//    painter.drawRect(BB.a.x(), BB.a.y(), BB.width(), BB.height());*/
// }

void VisionTab::drawEdgeDirections (QImage *image, bool top)
{
   // TODO: Re-add REFACTOR17

   /*int density = TOP_SALIENCY_DENSITY;
   if (!top) density = BOT_SALIENCY_DENSITY;

   QPainter painter (image);

   const int rect_size = density;
   const int r = rect_size / 2;

   int dx, dy;
   double mag, theta;
   int h, s, v;

   QPoint p1, p2;

   for (int row = 0; row < IMAGE_ROWS; row += rect_size) {
      for (int col = 0; col < IMAGE_COLS; col += rect_size) {
         Point temp = vision->topSaliency.asFovea().edge
                  (col / TOP_SALIENCY_DENSITY, row / TOP_SALIENCY_DENSITY);
         if (!top) {
            temp = vision->topSaliency.asFovea().edge
                  (col / BOT_SALIENCY_DENSITY, row / BOT_SALIENCY_DENSITY);
         }
         const Point &e = temp;

         dx = e[0];
         dy = e[1];

         theta = atan2 (dy, dx);
         if (theta < 0) {
            theta = 2 * M_PI + theta;
         }

         mag = hypot (dy, dx);

         h = (theta * 180) / (2 * M_PI);
         s = 255;
         v = mag < 255 ? mag : 255;

         painter.setPen (QColor ("black"));

         p1 = QPoint (col + r, row + r);
         p2 = QPoint (p1.x () + r * cos (theta),
                      p1.y () + r * sin (theta));

         painter.drawLine (p1, p2);
      }
   }*/
}


void VisionTab::drawOverlays(QImage *image, VisionInfoOut info_out, bool top) {
   /****************************************************************
    * Internal vision information not part of the public interface *
    ****************************************************************/

   // TODO: Re-add REFACTOR17
   OverlayPainter painter;
   painter.begin(image);

   /*OverlayPainter painter;
   painter.begin(image);
   std::vector<std::vector<Point> >::iterator g;
   std::vector<Point>::iterator gp;
   	      for (g  = vision->goalDetection.goalPostLines.begin();
   	           g != vision->goalDetection.goalPostLines.end();
   	           ++ g)
   	      {
   	    	 for(gp = (*g).begin();gp != (*g).end();++gp){
   	    		 painter.drawPoint (*gp, "red");
   	    	 }
   	      }

   if (!top) painter.translate(0, -TOP_IMAGE_ROWS);


   */

    if (checkFieldFeatures->isChecked()) {
        for (std::vector<FieldFeatureInfo>::iterator it = info_out.features.begin(); it != info_out.features.end(); ++it) {
            painter.drawFieldFeatureOverlay((*it), Qt::red);
        }
    }
    /*
   if (checkFieldFeatures->isChecked()) {


      std::vector<FieldLinePointInfo>::iterator i;
      for (i  = vision->fieldLineDetection.fieldLinePoints.begin();
           i != vision->fieldLineDetection.fieldLinePoints.end();
           ++ i)
      {
         //painter.drawPoint (i->p, "red");
      }

      for (uint16_t c = 0; c < vision->fieldLineDetection.linePoints.size(); c++)
      {
         for (i  = vision->fieldLineDetection.linePoints[c].begin ();
              i != vision->fieldLineDetection.linePoints[c].end   ();
              ++ i)
         {
            if (c == 0) painter.drawPoint (i->p, "purple");
            if (c == 1) painter.drawPoint (i->p, "red");
            if (c == 2) painter.drawPoint (i->p, "orange");
            if (c >= 3) painter.drawPoint (i->p, "blue");
         }
      }

      for (i  = vision->fieldLineDetection.circlePoints.begin ();
           i != vision->fieldLineDetection.circlePoints.end   ();
           ++ i)
      {
         painter.drawPoint (i->p, "black");
      }

      std::vector<FieldFeatureInfo>::const_iterator j;
      for (j  = vision->fieldLineDetection.fieldFeatures.begin ();
           j != vision->fieldLineDetection.fieldFeatures.end   ();
           ++ j)
      {
         if (j->type == FieldFeatureInfo::fPenaltySpot) {
            painter.drawFieldFeatureOverlay(*j, "green");
         }
      }


   }

   if (checkBall->isChecked()) {
      std::vector<Point> &points = vision->ballDetection.maximums;
      std::vector<Point>::iterator i;
      for (i  = points.begin(); i != points.end(); ++ i) {
         painter.drawPoint(*i, "orange");
      }

      points = vision->ballDetection.darkSpots;
      for (i  = points.begin(); i != points.end(); ++ i) {
         painter.drawPoint(*i, "red");
      }

      points = vision->ballDetection.clusterCentres;
      for (i  = points.begin(); i != points.end(); ++ i) {
         painter.drawPoint(*i, "purple");
      }

      points = vision->ballDetection.ccdCentres;
      for (i  = points.begin(); i != points.end(); ++ i) {
         painter.drawPoint(*i, "blue");
      }
   }

   if (checkFieldEdges->isChecked()) {
      std::vector<Point>::iterator i;
      for (i  = vision->fieldBoundaryDetection.boundaryPointsTop.begin ();
           i != vision->fieldBoundaryDetection.boundaryPointsTop.end   ();
           ++ i)
      {
            painter.drawPoint(*i, "#00ff00");
      }
      for (i  = vision->fieldBoundaryDetection.boundaryPointsBot.begin ();
           i != vision->fieldBoundaryDetection.boundaryPointsBot.end   ();
           ++ i)
      {
            painter.drawPoint(*i, "#00ff00");
      }
   }



   if (checkRobotsDebug->isChecked()) {
	   const std::vector<PossibleRobot> &possibleRobots = (checkBottomImage->isChecked())
			   ? vision->robotDetection._botPossibleRobots
			   : vision->robotDetection._topPossibleRobots;

	   const std::vector<int> &fieldBoundaries = (checkBottomImage->isChecked())
			   ? vision->robotDetection._botFieldEdges
			   : vision->robotDetection._topFieldEdges;

	   const int density = (checkBottomImage->isChecked()) ? BOT_SALIENCY_DENSITY : TOP_SALIENCY_DENSITY;

	   const int rowOffset = (top) ? 0 : TOP_IMAGE_ROWS;


	   QPainterPath startOfScanPath2;
	   for (unsigned int i = 0; i < fieldBoundaries.size(); ++i) {
		   if (i == 0) {
			   startOfScanPath2.moveTo(0, fieldBoundaries[0]);
		   } else {
			   startOfScanPath2.lineTo(i * density, (fieldBoundaries[i]) * density + rowOffset);
		   }
	   }
	   painter.drawLinePath(startOfScanPath2, Qt::blue);

	   for (unsigned int i = 0; i < possibleRobots.size(); ++i) {
		   drawBoundingBox(painter, possibleRobots[i].region, Qt::yellow, density, rowOffset);
	   }
   }

   if (checkFeetDebug->isChecked()){
	   for (std::vector<Point>::iterator it = vision->feetDebug.begin(); it != vision->feetDebug.end(); ++it){
	      painter.drawPoint(*it, QColor("red"));
	   }

   }
   painter.end();*/



   /****************************************************************
    * Public Vision Interface                                      *
    ****************************************************************/

   // TODO: Re-add REFACTOR17
   /*std::vector<FootInfo>	        feet_boxes	    = NULL;
   std::vector<BallInfo>            balls           = NULL;
   std::vector<PostInfo>            posts           = NULL;
   std::vector<RobotInfo>           robots          = NULL;
   std::vector<FieldBoundaryInfo>   fieldBoundaries = readFrom(vision, fieldBoundaries);
   std::vector<FieldFeatureInfo>    fieldFeatures   = NULL;
   std::vector<Ipoint>              landmarks       = readFrom(vision, landmarks);
   std::vector<RegionI>             regions         = readFrom(vision, regions);

   std::pair<int, int> horizon = vision->convRR.pose.getHorizon();

   if (checkHorizon->isChecked()) {
      horizon_p = &horizon;
   }

   if (checkBall->isChecked()) {
      balls = &vision->balls;
   }

   if (checkFeet->isChecked()) {
      feet_boxes = &vision->feet_boxes;
   }

   if (checkRobots->isChecked()) {
      robots = &vision->robots;
   }

   if (checkGoals->isChecked()) {
      posts = &vision->posts;
   }

   if (checkFieldEdges->isChecked()) {
      fieldBoundaries = &vision->fieldBoundaries;
   }

   if (checkFieldFeatures->isChecked()) {
      fieldFeatures = &vision->fieldFeatures;
   }

   QPaintDevice* t = (top) ? image : 0;
   QPaintDevice* b = (top) ? 0 : image;

   drawOverlaysGeneric(t,
                       b,
                       horizon_p,
                       feet_boxes,
                       balls,
                       posts,
                       robots,
                       fieldBoundaries,
                       fieldFeatures,
                       NULL,
                       regions,
                       1);
   */
   return;
}

// TODO(brockw): see if this can be genericized into tab.cpp, so it's not in
// every tab
void VisionTab::newNaoData(NaoData *naoData)
{
   // TODO: Re-add REFACTOR17
    if (!naoData || !naoData->getCurrentFrame().blackboard) {
        imagePixmap.fill(Qt::darkGray);
        camLabel->setPixmap(imagePixmap);
        topFrame = NULL;
    } else {
        Blackboard *blackboard = naoData->getCurrentFrame().blackboard;
        if (((topFrame = readFrom(vision, topFrame)) != NULL) && ((botFrame = readFrom(vision, botFrame)) != NULL)) {
            if (parent->currentIndex() == parent->indexOf(this)) {
                info_in.pose = readFrom(motion, pose);
                info_in.top_camera_settings = readFrom(vision, topCameraSettings);
                info_in.bot_camera_settings = readFrom(vision, botCameraSettings);
                info_in.top_frame = readFrom(vision, topFrame);
                info_in.bot_frame = readFrom(vision, botFrame);

                topFrame = readFrom(vision, topFrame);
                botFrame = readFrom(vision, botFrame);
                redraw();
            }
        }
    }
   /*nnmc_loaded = vision->nnmc_top.isLoaded() && vision->nnmc_bot.isLoaded();

   if (!naoData || !naoData->getCurrentFrame().blackboard) {
      imagePixmap.fill(Qt::darkGray);
      camLabel->setPixmap(imagePixmap);
      topFrame = NULL;
   } else {
      Blackboard *blackboard = naoData->getCurrentFrame().blackboard;
      if (((topFrame = readFrom(vision, topFrame)) != NULL) &&
          ((botFrame = readFrom(vision, botFrame)) != NULL))
         if (parent->currentIndex() == parent->indexOf(this)) {
            // added to allow FootDetection to work.
            vision->convRR.pose = readFrom(motion, pose);
            int behaviourReadBuf = readFrom(behaviour, readBuf);
            vision->whichCamera = readFrom(behaviour, request[behaviourReadBuf].whichCamera);
            vision->convRR.findEndScanValues();
            SensorValues values = readFrom(motion, sensors);
            vision->convRR.updateAngles(values);

            vision->robotDetection._sonar = readFrom(kinematics, sonarFiltered);

            RoboCupGameControlData gameData = readFrom(gameController, data);
            vision->goalMatcher.state = gameData.state;
            vision->goalMatcher.secondaryState = gameData.secondaryState;
            vision->goalMatcher.robotPos = readFrom(localisation, robotPos);

            redraw();
         }
   }*/
}


void VisionTab::redrawSlot() {
   redraw();
}

bool VisionTab::eventFilter(QObject *object, QEvent *event) {
   if ((object == camLabel) && (event->type() == QEvent::MouseButtonPress)) {
      return classifyMouseEvent(static_cast<QMouseEvent*>(event));
   } else if ((object == camLabel) && (event->type() == QEvent::Wheel)) {
      return classifyWheelEvent(static_cast<QWheelEvent*>(event));
   } else {
      return false;
   }
}

bool VisionTab::classifyMouseEvent(QMouseEvent *e) {
   if (e->button() == Qt::RightButton) {
      QString fileName = QFileDialog::getSaveFileName(this, "Save image");
      if (fileName != "") {
         lastRendering.save(fileName);
      }
   }
   return true;
}

bool VisionTab::classifyWheelEvent(QWheelEvent *e) {
   bool top = !checkBottomImage->isChecked();
   int oldZoomLog = zoomLog;
   zoomLog += e->delta()/100;

   if (zoomLog < 0) zoomLog = 0;

   if (zoomLog > ZOOM_LIMIT) zoomLog = ZOOM_LIMIT;

   zoomLevel = 1u<<zoomLog;

   mouseX = e->x();
   mouseY = e->y();

   if (top) {
      mouseX*=2;
      mouseY*=2;
   }

   if (zoomLog != oldZoomLog)  redraw();
   return true;
}

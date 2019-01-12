#pragma once

#include <QMenuBar>
#include <QWidget>
#include <QObject>
#include <QEvent>
#include <QGridLayout>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QLabel>
#include <QCheckBox>
#include <QRadioButton>
#include <QPushButton>
#include <QTabWidget>
#include <cstdio>
#include "tabs/tab.hpp"
#include "mediaPanel.hpp"
#include "perception/vision/Region.hpp"
#include "types/VisionInfoIn.hpp"
#include "types/VisionInfoOut.hpp"

class Fovea;


enum FoveaFeature{COLOUR, GREY, EDGE, EDGE_MAGNITUDE};
/*
 * This contains the vision debuging tab.
 */
class VisionTab : public Tab {
   Q_OBJECT
   public:
      VisionTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);

      void loadNnmcFile(WhichCamera cam, const char *f);

   private:
      void init();
      void initMenu(QMenuBar *menuBar);

      // Zoom stuff
      static const int ZOOM_LIMIT = 6;
      int zoomLevel, zoomLog;
      int mouseX, mouseY;
      int prevMouseX, prevMouseY, prevZoomLevel;


      QMenu *visionMenu;
      // TODO: REFACTOR17
      QAction *reloadBothNnmcAct;
      QAction *loadBotNnmcAct;
      QAction *loadTopNnmcAct;
      QAction *loadGoalMapAct;
      QAction *loadVisualWordsAct;
      QAction *loadAllAct;

      /* objects that the debug info is drawn to */
      QPixmap imagePixmap;
      QLabel *camLabel;

      QGridLayout *layout;

      /* Image selection radio buttons */
      QRadioButton *radioImage;
      QRadioButton *radioSaliency;
      QRadioButton *radioSaliencyEdge;
      QRadioButton *radioSaliencyGrey;

      /* checkboxes */
      QCheckBox *checkBottomImage;
      QCheckBox *checkEdgeDirections;
      QCheckBox *checkEdgeOrientations;
      QCheckBox *checkPatches;
      QCheckBox *checkLowLines;

      QCheckBox *checkHorizon;
      QCheckBox *checkBall;
      QCheckBox *checkFeet;
      QCheckBox *checkFeetDebug;
      QCheckBox *checkGoals;
      QCheckBox *checkRobots;
      QCheckBox *checkRobotsDebug;
      QCheckBox *checkFieldEdges;
      QCheckBox *checkFieldFeatures;

      QImage lastRendering;

      /* Re-draw the image box from current frame. */
      void redraw();

      /*  Draw the image on top of a pixmap */
      void drawFovea          (QImage *image, const Fovea &fovea, FoveaFeature feature);
      void drawImage          (QImage *image, bool top = true);
      void drawSaliency       (QImage *image, bool top = true);
      void drawSaliencyRegion (QImage *image, const RegionI& region);

      void drawEdgeDirections (QImage *image, bool top = true);

      /* Draw the overlays on top of that pixmap  */
      void drawOverlays(QImage *image, VisionInfoOut info_out, bool top);


      /* event stuff */
      void mouseMoveEvent(QMouseEvent * event);

      QPoint mousePosition;

      bool eventFilter(QObject *object, QEvent *event);

      bool classifyMouseEvent(QMouseEvent *e);
      bool classifyWheelEvent(QWheelEvent *e);

      bool nnmc_loaded;

      VisionInfoIn info_in;


   public slots:
      void newNaoData(NaoData *naoData);

      // TODO: REFACTOR17
      /*void reloadBothNnmc();
      void loadTopNnmc();
      void loadBotNnmc();
      void loadGoalMap();
      void loadVisualWords();
      void loadAll();*/

      void redrawSlot();
};


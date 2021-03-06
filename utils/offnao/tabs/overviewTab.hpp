#pragma once

#include <QTabWidget>

#include <QMenuBar>
#include <QWidget>
#include <QObject>
#include <QEvent>
#include <QGridLayout>
#include <QPixmap>
#include <QLabel>
#include <QPainter>
#include <QProcess>
#include <QColor>
#include <QTreeWidgetItem>
#include <QTreeWidget>

#include <cstdio>
#include <deque>

#include "tabs/tab.hpp"
#include "tabs/variableView.hpp"
#include "perception/vision/other/YUV.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "utils/Logger.hpp"
#include "fieldView.hpp"
#include "mediaPanel.hpp"

class Vision;

class Blackboard;

/*
 * This is the default/initial tab that shows on entering Off-Nao.
 * As the name suggests it gives an overview of most of the important
 * aspects of the nao. Good for general gameplay watching etc.
 *
 * Also will be used for localization debugging unless the localization
 * people need more info.
 */
class OverviewTab : public Tab {
   Q_OBJECT
   QProcess scpNNMC;
   public:
      OverviewTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision
                  );
      QPixmap *renderPixmap;
      QLabel *renderWidget;
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void redraw();

      /*  Draw the image on top of a pixmap */
      void drawImage(QImage *topImage, QImage *botImage);
      void drawOverlays(QPixmap *topImage, QPixmap *botImage);

      QString naoName;

      QGridLayout *layout;

      QMenu *vision2017Menu;
      QAction *saveTopNNMCAct;
      QAction *saveBotNNMCAct;
      QAction *loadTopNnmcAct;
      QAction *loadBotNnmcAct;

      /* These variables are used to present the debug variables from the nao*/
      VariableView variableView;
      FieldView fieldView;

      /* Variables for vision */
      QPixmap topImagePixmap;
      QLabel *topCamLabel;
      QPixmap botImagePixmap;
      QLabel *botCamLabel;

      // Data
      Colour topSaliency[TOP_SALIENCY_ROWS][TOP_SALIENCY_COLS];
      Colour botSaliency[BOT_SALIENCY_ROWS][BOT_SALIENCY_COLS];
      Blackboard *blackboard;

   public slots:
      void newNaoData(NaoData *naoData);
      void loadTopNnmc();
      void loadBotNnmc();
      void saveTopNNMC();
      void saveBotNNMC();
      void setNao(const QString &naoName);
};


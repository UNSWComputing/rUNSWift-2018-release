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
#include <QColor>
#include <QTreeWidgetItem>
#include <QTreeWidget>
#include <QLabel>

#include <cstdio>
#include <deque>

#include "tabs/tab.hpp"
#include "tabs/variableView.hpp"
#include "tabs/plots.hpp"
#include "perception/vision/other/YUV.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "utils/Logger.hpp"
#include "fieldView.hpp"
#include "mediaPanel.hpp"

class Vision;

class GraphTab : public Tab {
   Q_OBJECT
   public:
      GraphTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void updatePlots(SensorValues s, Odometry o, bool left=false);
      void updatePlots(std::vector<SensorValues> s, std::vector<Odometry> o, bool left=false);
      QGridLayout *layout;
      MultiPlot* accPlots[3];
      MultiPlot* gyrPlots[3];
      MultiPlot* anglePlots[2];
      MultiPlot* odomPlot;
      int last_frame;
   public slots:
      void newNaoData(NaoData *naoData);
      void readerClosed();
};


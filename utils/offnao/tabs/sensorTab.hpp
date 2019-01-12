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
#include <QPushButton>

#include <cstdio>
#include <deque>

#include "tabs/tab.hpp"
#include "tabs/variableView.hpp"
#include "perception/vision/other/YUV.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "utils/Logger.hpp"
#include "fieldView.hpp"
#include "mediaPanel.hpp"
#include "utils/body.hpp"


#define POS_FILE_HEADER "# HY   HP   LSP  LSR  LEY  LER  LWY  LH   LHYP LHR  LHP  LKP  LAP  LAR  RHR  RHP  RKP  RAP  RAR  RSP  RSR  REY  RER  RWY  RH   DUR"
#define POS_FILE_STIFFNESS "$ 0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5  0.5"

#define COL_WIDTH 5

class Vision;

class SensorTab : public Tab {
   Q_OBJECT
   public:
      SensorTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void updateDataView(Blackboard *blackboard);
      QGridLayout *layout;
      QLabel *jointValueLabel;
      QLabel *sensorValueLabel;
      QLabel *sonarValueLabel;
      QPushButton *genPos;
      std::vector<int> joints;
   public slots:
      void newNaoData(NaoData *naoData);
      void generatePosFile();
};


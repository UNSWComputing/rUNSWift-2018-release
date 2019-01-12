#pragma once

#include <QtGui/QMainWindow>
#include <QTreeWidget>
#include <QTabWidget>
#include <QLabel>
#include <QFileDialog>

#include <vector>
#include <map>
#include <string>

#include "naoData.hpp"
#include "mediaPanel.hpp"

#include "readers/reader.hpp"
#include "readers/networkReader.hpp"
#include "readers/dumpReader.hpp"

#include "tabs/cameraTab.hpp"
#include "tabs/classifier.hpp"
#include "tabs/overviewTab.hpp"
#include "tabs/nnmcTab.hpp"
#include "tabs/calibrationTab.hpp"
#include "tabs/visionTab.hpp"
#include "tabs/localisationTab.hpp"
#include "tabs/sensorTab.hpp"
#include "tabs/cameraPoseTab.hpp"
#include "tabs/graphTab.hpp"
#include "tabs/walkTab.hpp"
#include "tabs/zmpTab.hpp"
#include "tabs/controlTab.hpp"
#include "tabs/LogsTab.hpp"
//#include "tabs/surfTab.hpp"
#include "tabs/icpTab.hpp"
#include "tabs/teamTab.hpp"
#include "tabs/teamBallTab.hpp"


#include "perception/vision/other/YUV.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "perception/vision/Vision.hpp"
#include "perception/localisation/LocalisationDefs.hpp"

#include "utils/Logger.hpp"

#include "ui_ConnectionBar.h"

class Tab;

namespace Ui {
   class Visualiser;
}

/*
 * This is the main window. This holds data at the root level, such as
 * tabs, status bar and the file menubar.
 */
class Visualiser : public QMainWindow {
   /* Allows main to load dump files */
   friend int main (int, char **);

   Q_OBJECT

   public:
      explicit Visualiser(QWidget *parent = 0);
      ~Visualiser();

      public slots:
         /* This slot is received when the reader aquires more data   */
         void newNaoData(NaoData *naoData);

      /* Temp function for demo purposes.  */
      void startRecording(QAction* action);

      /**
       * lets the user select a file to open, then loads the file
       */
      void openFileDialogue();
      void openFile(const QString &path);

      /**
       * lets the user select a file to save, then saves the file
       */
      void saveFileDialogue();
      void saveWhiteboardFileDialogue();
      void saveFile(const QString &path);

      /**
       * lets the user select a folder to save images from a dump file (in bmp)
       */
      void extractRawImagesDialogue();
      void extractRawImages(const QString &path);

      /**
       * lets the user select a folder to save ROIs from a dump file.
       */
      void extractROIsDialogue();
      void extractROIs(const QString &path);

      /* Called when a tab is changed. Currently is used to refresh
       * the current tab when you first switch to it.
       */
      void currentTabChanged(int tabIndex);

      void connectToNao();
      void connectToNao(const QString &naoName);
      void writeToNao(QAbstractButton *qab);

      void keyPressEvent(QKeyEvent *event);
      void keyReleaseEvent(QKeyEvent *event);

      void disconnectFromNao();

      /**
       * sends a command line option to the nao
       */
      void commandLineString();


      void close();

   signals:
      /* Used to tell a widget to refresh after a certain event occurs.
       * Currently is used to redraw a tab when we switch to it
       */
      void refreshNaoData();

      /**
       * sends a command line option to the nao
       */
      void sendCommandLineString(const QString &);

      void readerClosed();

   private:
      Ui::Visualiser *ui;
      std::vector<Tab*> tabVector;

      /* The reader. This could be any of the readers subclasses. This means that in the end the gui doesn't really
         care weather the data it is receiving is coming from disk or over wireless */
      Reader *reader;

      /* A pointer to the naoData */
      NaoData *naoData;

      QTabWidget *tabs;

      /* Tabs */
      OverviewTab *overviewTab;
      NNMCTab *nnmcTab;
      CalibrationTab *calibrationTab;
      VisionTab *visionTab;
      LocalisationTab *localisationTab;
      CameraTab *cameraTab;
      SensorTab *sensorTab;
      CameraPoseTab *cameraPoseTab;
      GraphTab *graphTab;
      WalkTab *walkTab;
      ZMPTab *zmpTab;
      ControlTab *controlTab;
      LogsTab *logsTab;
      //SurfTab *surfTab;
      ICPTab *icpTab;
      TeamTab *teamTab;
      TeamBallTab *teamBallTab;

      /* initializes the general menu's. i.e. menus that are
       * not specific to a tab.
       */
      void initMenu();


      /* Variables for the file menu */
      QMenu *fileMenu;
      QAction *loadAct;
      QAction *saveAsAct;
      QAction *exitAct;

      /* used for connecting to naos */
      QDockWidget *connectionBar;
      Ui::ConnectionBar cb;
      OffNaoMask_t transmissionMask(const QAbstractButton *);

      /* This holds the global media panel consisting of
       * record/play/stop/pause buttons
       */
      MediaPanel *mediaPanel;

      Vision *vision;

      void setUpReaderSignals(Reader *reader);

      /* Used for sending the deselect event to the correct tab */
      int lastTabIndex;

      /**
       * disconnects a reader, and connects a new one (saving the old data)
       *
       * @param ReaderClass the new reader type to create
       * @param Void the argument type for the constructor
       * @param constructorArg1 the argument for the constructor
       */
      template<class ReaderClass, typename Void>
      bool reconnect(const Void &constructorArg1);
};

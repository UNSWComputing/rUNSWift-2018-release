#pragma once

#include "tabs/tab.hpp"
#include "mediaPanel.hpp"
#include "utils/FieldPainter.hpp"
#include "utils/FieldObject.hpp"
#include "types/AbsCoord.hpp"

#include <vector>

#include <QGroupBox>
#include <QMenuBar>
#include <QGridLayout>
#include <QPixmap>
#include <QLabel>
#include <QImage>

#include <cstdio>
#include <deque>



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
class TeamBallTab : public Tab {
   Q_OBJECT
   public:
      TeamBallTab(QTabWidget *parent, QMenuBar *menuBar
                  );
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void redraw();

      QTransform transform;
      void setTransform(int device_width, int device_height);

      QGridLayout *layout;

      /* These variables are used to present the debug variables from the nao*/
      QLabel *fieldLabel;
      QPixmap imagePixmap;
      QImage image;

      // Data
      Blackboard *blackboard;

   public slots:
      void newNaoData(NaoData *naoData);
};

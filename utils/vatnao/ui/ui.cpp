#include "ui.hpp"

#include <QtGui>
#include <QPushButton>
#include <QObject>
#include <QLabel>
#include <QPixmap>

#include "uiElements/imageView.hpp"
#include "uiElements/debugFrame.hpp"

UI::UI(int argc, char *argv[], AppAdaptor *appAdaptor):
    app(argc, argv),
    vatnaoManager(appAdaptor)
{}

int UI::exec(){
    QWidget window;

    ImageView topImageView;
    ImageView botImageView;

    QPushButton btnPPrev("<<");
    QPushButton btnPrev("<-");
    QPushButton btnNext("->");
    QPushButton btnNNext(">>");

    QObject::connect(&btnPPrev, SIGNAL(clicked()), &vatnaoManager, SLOT(prevTenFrames()));
    QObject::connect(&btnPrev, SIGNAL(clicked()), &vatnaoManager, SLOT(prevFrame()));
    QObject::connect(&btnNext, SIGNAL(clicked()), &vatnaoManager, SLOT(nextFrame()));
    QObject::connect(&btnNNext, SIGNAL(clicked()), &vatnaoManager, SLOT(nextTenFrames()));
    QObject::connect(&vatnaoManager, SIGNAL(topImageChanged(QPixmap)), &topImageView, SLOT(setPixmap(QPixmap)));
    QObject::connect(&vatnaoManager, SIGNAL(botImageChanged(QPixmap)), &botImageView, SLOT(setPixmap(QPixmap)));

    DebugFrame debug_frame(vatnaoManager);

    QObject::connect(&vatnaoManager, SIGNAL(debugInfoChanged(AppStatus&, RgbImg*, RgbImg*)),
                     &debug_frame, SLOT(updateFrame(AppStatus&, RgbImg*, RgbImg*)));

    QGridLayout *layout = new QGridLayout;

    layout->addWidget(&topImageView, 0, 0, 1, 8);
    layout->addWidget(&botImageView, 1, 0, 1, 8);
    layout->addWidget(&btnPPrev, 2, 0);
    layout->addWidget(&btnPrev, 2, 1, 1, 3);
    layout->addWidget(&btnNext, 2, 4, 1, 3);
    layout->addWidget(&btnNNext, 2, 7);
    layout->addWidget(&debug_frame, 0, 8, 3, 3);

    window.setLayout(layout);
    window.show();

    return app.exec();
}

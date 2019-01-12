#pragma once

#include <QtCore/qglobal.h>
#include <QtGui/QApplication>
#include <QImage>

#include "vatnaoManager.hpp"

#include "../app/appAdaptor.hpp"

class UI {
    public:
        UI(int argc, char *argv[], AppAdaptor *appAdaptor);

        // Call to start the UI.
        int exec();

    private:
        QApplication app;
        VatnaoManager vatnaoManager;
};

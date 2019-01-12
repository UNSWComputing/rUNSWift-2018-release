#pragma once

#include <QObject>
#include <QPixmap>

#include <string>

#include "../app/appAdaptor.hpp"

class VatnaoManager:public QObject{
    Q_OBJECT
    public:
        VatnaoManager(AppAdaptor *a);
        ~VatnaoManager();

        QImage *topImage;
        QImage *botImage;

        AppStatus getAppStatus();

        void updateBooleanOption(string option_name, bool value);
        void updateStringOption(string option_name, string option);
        void updateNumericOption(string option_name, double value);
        void updateRegionIndex(int change);
        void updateSubregionIndex(int change);

        void reprocess();
    signals:
        void topImageChanged(QPixmap topImage);
        void botImageChanged(QPixmap botImage);
        void debugInfoChanged(AppStatus &app_status, RgbImg* prevRegion, RgbImg* anRegion);
    public slots:
        void nextFrame();
        void nextTenFrames();
        void prevFrame();
        void prevTenFrames();

        void nextTenRegions();
        void nextRegion();
        void prevRegion();
        void prevTenRegions();

        void nextTenSubRegions();
        void nextSubRegion();
        void prevSubRegion();
        void prevTenSubRegions();
    private:

        void refreshImages();

        AppAdaptor *appAdaptor;
        VatnaoQuery vatnao_query_;
};

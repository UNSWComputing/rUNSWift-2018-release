#pragma once

#include <QObject>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QPixmap>
#include <QRadioButton>
#include <QImage>
#include <QImageWriter>
#include <QDebug>
#include <QDir>
#include <QFile>

#include "../app/appAdaptor.hpp"

#include "../../robot/perception/vision/detector/BallDetector.hpp"

#include "frameTools/readImage.hpp"
#include "frameTools/readBoundingBoxes.hpp"

#include <string>
#include <ctime>

class VatnaoManager:public QObject{
    Q_OBJECT
    public:
        VatnaoManager(AppAdaptor *a);
        ~VatnaoManager();

        QImage *topImage;
        QImage *botImage;

        QImage *regionRawImage;
        QImage *regionSaliencyImage;

        QRadioButton *radioRaw;
        QRadioButton *radioBinary;

        QRadioButton *radioCircle;
        QRadioButton *radioBlack;
        QRadioButton *radioBlob;
        QRadioButton *radioCombo;

        QCheckBox *checkBoxCandidatePoints;
        QCheckBox *checkBoxCircleCentres;
        QCheckBox *checkBoxCircleFit;

        QCheckBox *checkBoxRegions;
        QCheckBox *checkBoxRegionCentres;
        QCheckBox *checkBoxRegionTriangles;

        QCheckBox *checkBoxDownsampled;
        QCheckBox *checkBoxBallMetaData;
        QCheckBox *checkBoxATInput;
        
        QLineEdit *windowSize;
        QLineEdit *percentage;
        QLineEdit *ballDataFileName;
        QLineEdit *ballFinderNumTrials;

        QLabel *resultInfo;
        

    signals:
        void topImageChanged(QPixmap topImage);
        void botImageChanged(QPixmap botImage);
        void regionImageChanged(QPixmap regionRawImage);
        void regionImageSaliencyChanged(QPixmap regionRawImage);
    public slots:
        void nextFrame();
        void nextTenFrames();
        void prevFrame();
        void prevTenFrames();
        void nextRegion();
        void nextTenRegions();
        void prevRegion();
        void prevTenRegions();

        void nextROI();
        void nextTenROIs();
        void prevROI();
        void prevTenROIs();
 
        void saliencySelectionMade();
        void saliencyROISelectionMade();
        void showMetaData();

        void saveSaliencyT();
        void saveSaliencyF();

        void FindBall();
            
    private:

        int prev_region_counter_;
        int region_counter_;
        int ball_region_counter_;

        void refreshImages();
        void refreshRegionImages();
        void refreshRegionROIImages();

        AppAdaptor *appAdaptor;

        std::vector <BallDetectorVisionBundle> ball_regions_;

        QPainter *painter;

        SaliencyType t;
        bool downSampled;
        bool ATInputs;
        bool candidatePoints;
        bool circleCentres;
        bool circleFit;
        bool regions;
        bool regionCentres;
        bool regionTriangles;
};

#include "ui.hpp"

#include <QtGui>
#include <QCheckBox>
#include <QGroupBox>
#include <QPushButton>
#include <QRadioButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QObject>
#include <QLabel>
#include <QPixmap>
#include <QSizePolicy>

#include "uiElements/imageView.hpp"

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

    /*
     * Region visualiser
     */

    QGroupBox *groupBoxRegions = new QGroupBox ("Regions");
    QBoxLayout *vboxRegions = new QVBoxLayout ();

    groupBoxRegions->setLayout (vboxRegions);

    // Buttons

    QGroupBox *groupBoxRegionButtons = new QGroupBox ();
    QHBoxLayout *regionButtons = new QHBoxLayout ();

    QPushButton btnPPrevRegion("<<");
    QPushButton btnPrevRegion("<-");
    QPushButton btnNextRegion("->");
    QPushButton btnNNextRegion(">>");

    regionButtons->addWidget (&btnPPrevRegion);
    regionButtons->addWidget (&btnPrevRegion);
    regionButtons->addWidget (&btnNextRegion);
    regionButtons->addWidget (&btnNNextRegion);

    groupBoxRegionButtons->setLayout (regionButtons);

    vboxRegions->addWidget(groupBoxRegionButtons);

    QObject::connect(&btnPPrevRegion, SIGNAL(clicked()), &vatnaoManager, SLOT(prevTenRegions()));
    QObject::connect(&btnPrevRegion, SIGNAL(clicked()), &vatnaoManager, SLOT(prevRegion()));
    QObject::connect(&btnNextRegion, SIGNAL(clicked()), &vatnaoManager, SLOT(nextRegion()));
    QObject::connect(&btnNNextRegion, SIGNAL(clicked()), &vatnaoManager, SLOT(nextTenRegions()));

    // Saliency type

    QGroupBox *groupBoxSaliency = new QGroupBox ("Saliency type");
    QGridLayout *saliencyLayout = new QGridLayout();

    vatnaoManager.radioRaw = new QRadioButton ("Raw Image");
    vatnaoManager.radioBinary = new QRadioButton ("Binary Image");

    vatnaoManager.radioRaw->setChecked(true);

    saliencyLayout->addWidget (vatnaoManager.radioRaw, 0, 0, 1, 1);
    saliencyLayout->addWidget (vatnaoManager.radioBinary, 0, 1, 1, 1);

    vatnaoManager.checkBoxCandidatePoints = new QCheckBox("Candidate Points");
    vatnaoManager.checkBoxCircleCentres = new QCheckBox("Circle Centres");
    vatnaoManager.checkBoxCircleFit = new QCheckBox("Circle Fit");

    vatnaoManager.checkBoxRegions = new QCheckBox("Regions");
    vatnaoManager.checkBoxRegionCentres = new QCheckBox("Region Centres");
    vatnaoManager.checkBoxRegionTriangles = new QCheckBox("Region Triangles");

    vatnaoManager.checkBoxDownsampled = new QCheckBox("Downsampled?");
    vatnaoManager.checkBoxBallMetaData = new QCheckBox("Show ball metadata?");
    vatnaoManager.checkBoxATInput = new QCheckBox("Input AT Values?");

    vatnaoManager.windowSize = new QLineEdit("");
    vatnaoManager.percentage = new QLineEdit("");

    vatnaoManager.checkBoxCandidatePoints->setChecked(false);
    vatnaoManager.checkBoxCircleCentres->setChecked(false);
    vatnaoManager.checkBoxCircleFit->setChecked(false);

    vatnaoManager.checkBoxRegions->setChecked(false);
    vatnaoManager.checkBoxRegionCentres->setChecked(false);
    vatnaoManager.checkBoxRegionTriangles->setChecked(false);

    vatnaoManager.checkBoxDownsampled->setChecked(true);
    vatnaoManager.checkBoxBallMetaData->setChecked(false);
    vatnaoManager.checkBoxATInput->setChecked(false);
    
    vatnaoManager.resultInfo = new QLabel();
    vatnaoManager.resultInfo->setFont(QFont("Monospace"));
    vatnaoManager.resultInfo->setText("Status");

    saliencyLayout->addWidget (vatnaoManager.checkBoxCandidatePoints, 3, 0, 1, 1);
    saliencyLayout->addWidget (vatnaoManager.checkBoxCircleCentres, 3, 1, 1, 1);
    saliencyLayout->addWidget (vatnaoManager.checkBoxCircleFit, 3, 2, 1, 1);

    saliencyLayout->addWidget (vatnaoManager.checkBoxRegions, 4, 0, 1, 1);
    saliencyLayout->addWidget (vatnaoManager.checkBoxRegionCentres, 4, 1, 1, 1);
    saliencyLayout->addWidget (vatnaoManager.checkBoxRegionTriangles, 4, 2, 1, 1);
    
    saliencyLayout->addWidget (vatnaoManager.checkBoxDownsampled, 5, 0, 1, 1);
    saliencyLayout->addWidget (vatnaoManager.checkBoxBallMetaData, 5, 1, 1, 1);
    saliencyLayout->addWidget (vatnaoManager.checkBoxATInput, 5, 2, 1, 1);

    saliencyLayout->addWidget (vatnaoManager.windowSize, 6, 1, 1, 1);
    saliencyLayout->addWidget (vatnaoManager.percentage, 6, 2, 1, 1);
    saliencyLayout->addWidget (vatnaoManager.resultInfo, 7, 0, 1, 1);
    

    groupBoxSaliency->setLayout (saliencyLayout);

    vboxRegions->addWidget(groupBoxSaliency);

    QObject::connect(vatnaoManager.radioRaw, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencySelectionMade()));
    QObject::connect(vatnaoManager.radioBinary, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencySelectionMade()));

    QObject::connect(vatnaoManager.checkBoxCandidatePoints, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencySelectionMade()));
    QObject::connect(vatnaoManager.checkBoxCircleCentres, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencySelectionMade()));
    QObject::connect(vatnaoManager.checkBoxCircleFit, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencySelectionMade()));

    QObject::connect(vatnaoManager.checkBoxRegions, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencySelectionMade()));
    QObject::connect(vatnaoManager.checkBoxRegionCentres, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencySelectionMade()));
    QObject::connect(vatnaoManager.checkBoxRegionTriangles, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencySelectionMade()));
    
    QObject::connect(vatnaoManager.checkBoxDownsampled, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencySelectionMade()));
    QObject::connect(vatnaoManager.checkBoxBallMetaData, SIGNAL(clicked()), &vatnaoManager, SLOT(showMetaData()));
    QObject::connect(vatnaoManager.checkBoxATInput, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencySelectionMade()));
    
    QObject::connect(vatnaoManager.windowSize, SIGNAL(returnPressed()), &vatnaoManager, SLOT(saliencySelectionMade()));
    QObject::connect(vatnaoManager.percentage, SIGNAL(returnPressed()), &vatnaoManager, SLOT(saliencySelectionMade()));

    // saliency saver

    QGroupBox *groupBoxSaliencySaver = new QGroupBox ("Saliency Saver");
    QGridLayout *saliencySaverLayout = new QGridLayout();

    vatnaoManager.ballDataFileName = new QLineEdit("");
    QPushButton btnT("T");
    QPushButton btnF("F");

    saliencySaverLayout->addWidget (vatnaoManager.ballDataFileName, 0, 0, 1, 1);
    saliencySaverLayout->addWidget (&btnT, 0, 1, 1, 1);
    saliencySaverLayout->addWidget (&btnF, 0, 2, 1, 1);

    groupBoxSaliencySaver->setLayout(saliencySaverLayout);

    vboxRegions->addWidget(groupBoxSaliencySaver);


    // Ball Finder

    QGroupBox *groupBoxBallFinder = new QGroupBox ("Ball Finder");
    QGridLayout *ballFinderLayout = new QGridLayout();

    vatnaoManager.ballFinderNumTrials = new QLineEdit("");
    QPushButton btnFindBall("Find Ball");

    ballFinderLayout->addWidget (vatnaoManager.ballFinderNumTrials, 0, 0, 1, 1);
    ballFinderLayout->addWidget (&btnFindBall, 0, 1, 1, 1);

    groupBoxBallFinder->setLayout(ballFinderLayout);
    vboxRegions->addWidget(groupBoxBallFinder);


    QGroupBox *groupBoxSaliencyROI = new QGroupBox ("ROI type");
    QGridLayout *saliencyROILayout = new QGridLayout();
    
    vatnaoManager.radioCircle = new QRadioButton ("Circle ROI");
    vatnaoManager.radioBlack = new QRadioButton ("Black ROI");
    vatnaoManager.radioBlob = new QRadioButton ("Blob ROI");        
    vatnaoManager.radioCombo = new QRadioButton ("Combo ROI");

    vatnaoManager.radioCombo->setChecked(true);
    
    saliencyROILayout->addWidget (vatnaoManager.radioCircle, 0, 0, 1, 1);
    saliencyROILayout->addWidget (vatnaoManager.radioBlack, 0, 1, 1, 1);
    saliencyROILayout->addWidget (vatnaoManager.radioBlob, 1, 0, 1, 1);    
    saliencyROILayout->addWidget (vatnaoManager.radioCombo, 1, 1, 1, 1);

    groupBoxSaliencyROI->setLayout (saliencyROILayout);

    vboxRegions->addWidget(groupBoxSaliencyROI);

    QObject::connect(vatnaoManager.radioCircle, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencyROISelectionMade()));
    QObject::connect(vatnaoManager.radioBlack, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencyROISelectionMade()));
    QObject::connect(vatnaoManager.radioBlob, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencyROISelectionMade()));    
    QObject::connect(vatnaoManager.radioCombo, SIGNAL(clicked()), &vatnaoManager, SLOT(saliencyROISelectionMade()));

    QGroupBox *groupBoxROIButtons = new QGroupBox ();
    QHBoxLayout *roiButtons = new QHBoxLayout ();
    //QGroupBox *groupBoxRegions = new QGroupBox ("ROIS");
    

    QPushButton btnPPrevROI("<<");
    QPushButton btnPrevROI("<-");
    QPushButton btnNextROI("->");
    QPushButton btnNNextROI(">>");

    roiButtons->addWidget (&btnPPrevROI);
    roiButtons->addWidget (&btnPrevROI);
    roiButtons->addWidget (&btnNextROI);
    roiButtons->addWidget (&btnNNextROI);

    groupBoxROIButtons->setLayout (roiButtons);

    vboxRegions->addWidget(groupBoxROIButtons);

    QObject::connect(&btnPPrevROI, SIGNAL(clicked()), &vatnaoManager, SLOT(prevTenROIs()));
    QObject::connect(&btnPrevROI, SIGNAL(clicked()), &vatnaoManager, SLOT(prevROI()));
    QObject::connect(&btnNextROI, SIGNAL(clicked()), &vatnaoManager, SLOT(nextROI()));
    QObject::connect(&btnNNextROI, SIGNAL(clicked()), &vatnaoManager, SLOT(nextTenROIs()));



    // Image representation

    ImageView rawImageView;
    ImageView saliencyImageView;

    QSizePolicy spRaw(QSizePolicy::Preferred, QSizePolicy::Preferred);
    spRaw.setVerticalStretch(1);
    rawImageView.setSizePolicy(spRaw);
    vboxRegions->addWidget(&rawImageView);

    QSizePolicy spSaliency(QSizePolicy::Preferred, QSizePolicy::Preferred);
    spSaliency.setVerticalStretch(1);
    saliencyImageView.setSizePolicy(spSaliency);
    vboxRegions->addWidget(&saliencyImageView);

    QObject::connect(&vatnaoManager, SIGNAL(regionImageChanged(QPixmap)), &rawImageView, SLOT(setPixmap(QPixmap)));
    QObject::connect(&vatnaoManager, SIGNAL(regionImageSaliencyChanged(QPixmap)), &saliencyImageView, SLOT(setPixmap(QPixmap)));

    QObject::connect(&btnT, SIGNAL(clicked()), &vatnaoManager, SLOT(saveSaliencyT()));
    QObject::connect(&btnF, SIGNAL(clicked()), &vatnaoManager, SLOT(saveSaliencyF()));

    QObject::connect(&btnFindBall, SIGNAL(clicked()), &vatnaoManager, SLOT(FindBall()));


    /*
     * Set up layout
     */

    QGridLayout *layout = new QGridLayout;

    layout->addWidget(&topImageView, 0, 0, 1, 8);
    layout->addWidget(&botImageView, 1, 0, 1, 8);
    layout->addWidget(&btnPPrev, 2, 0);
    layout->addWidget(&btnPrev, 2, 1, 1, 3);
    layout->addWidget(&btnNext, 2, 4, 1, 3);
    layout->addWidget(&btnNNext, 2, 7);

    layout->addWidget(groupBoxRegions, 0, 9, 2, 1, Qt::AlignTop | Qt::AlignLeft);

    window.setLayout(layout);
    window.show();

    return app.exec();
}

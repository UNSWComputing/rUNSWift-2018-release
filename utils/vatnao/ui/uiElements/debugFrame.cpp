#include "debugFrame.hpp"

#include <QPushButton>
#include <QObject>
#include <vector>

#include <iostream>

#include <boost/lexical_cast.hpp>

#include "../../app/appStatus.hpp"
#include "../frameTools/readImage.hpp"

DebugFrame::DebugFrame(VatnaoManager& vatnao_manager, QWidget *parent):
    QFrame(parent),
    vatnao_manager_(vatnao_manager)
{
    parent_ = parent;
    AppStatus as = vatnao_manager_.getAppStatus();
    initControls(as);
}

DebugFrame::~DebugFrame() {
    clearOptionControls();
    delete rawRegion_;
    delete editedRegion_;
    delete btnRegionPrev_;
    delete btnRegionPPrev_;
    delete btnRegionNNext_;
    delete btnRegionNext_;
    delete btnSubRegionPrev_;
    delete btnSubRegionPPrev_;
    delete btnSubRegionNNext_;
    delete btnSubRegionNext_;
    delete optionsFrame_;
    delete optionsLayout_;
    delete layout_;
}

void DebugFrame::updateFrame(AppStatus &app_status,
                             RgbImg* previewRegion,
                             RgbImg* annotatedRegion)
{
    drawOptionControls(app_status);

    lblRegionIndex_->setText(QString(boost::lexical_cast<std::string>(app_status.region_index).c_str()));
    lblSubRegionIndex_->setText(QString(boost::lexical_cast<std::string>(app_status.subregion_index).c_str()));

    if (previewRegion != NULL) {
        QImage q_preview(previewRegion->getCols(), previewRegion->getRows(), QImage::Format_RGB32);
        RgbImgToQImage(previewRegion, &q_preview);
        rawRegion_->setPixmap(QPixmap::fromImage(q_preview));
    }
    if (annotatedRegion != NULL) {
        QImage q_annotated(annotatedRegion->getCols(), annotatedRegion->getRows(), QImage::Format_RGB32);
        RgbImgToQImage(annotatedRegion, &q_annotated);
        editedRegion_->setPixmap(QPixmap::fromImage(q_annotated));
    }

    debugMessage_->setPlainText(QString(app_status.debug_message.c_str()));
}

void DebugFrame::initControls(AppStatus &app_status) {

    rawRegion_    = new ImageView;
    editedRegion_ = new ImageView;

    btnRegionPPrev_ = new QPushButton("<<");
    btnRegionPrev_  = new QPushButton("<-");
    btnRegionNext_  = new QPushButton("->");
    btnRegionNNext_ = new QPushButton(">>");
    lblRegionIndex_ = new QLabel;

    btnSubRegionPPrev_ = new QPushButton("<<");
    btnSubRegionPrev_  = new QPushButton("<-");
    btnSubRegionNext_  = new QPushButton("->");
    btnSubRegionNNext_ = new QPushButton(">>");
    lblSubRegionIndex_ = new QLabel;

    QObject::connect(btnRegionPPrev_, SIGNAL(clicked()), &vatnao_manager_, SLOT(prevTenRegions()));
    QObject::connect(btnRegionPrev_, SIGNAL(clicked()), &vatnao_manager_, SLOT(prevRegion()));
    QObject::connect(btnRegionNext_, SIGNAL(clicked()), &vatnao_manager_, SLOT(nextRegion()));
    QObject::connect(btnRegionNNext_, SIGNAL(clicked()), &vatnao_manager_, SLOT(nextTenRegions()));

    QObject::connect(btnSubRegionPPrev_, SIGNAL(clicked()), &vatnao_manager_, SLOT(prevTenSubRegions()));
    QObject::connect(btnSubRegionPrev_, SIGNAL(clicked()), &vatnao_manager_, SLOT(prevSubRegion()));
    QObject::connect(btnSubRegionNext_, SIGNAL(clicked()), &vatnao_manager_, SLOT(nextSubRegion()));
    QObject::connect(btnSubRegionNNext_, SIGNAL(clicked()), &vatnao_manager_, SLOT(nextTenSubRegions()));

    optionsFrame_  = new QFrame;
    optionsLayout_ = new QGridLayout(optionsFrame_);

    drawOptionControls(app_status);

    debugMessage_ = new QTextEdit;
    debugMessage_->setReadOnly(true);

    layout_ = new QGridLayout(this);
    layout_->addWidget(lblRegionIndex_, 0, 0);
    layout_->addWidget(btnRegionPPrev_, 0, 1);
    layout_->addWidget(btnRegionPrev_,  0, 2, 1, 3);
    layout_->addWidget(btnRegionNext_,  0, 5, 1, 3);
    layout_->addWidget(btnRegionNNext_, 0, 8);

    layout_->addWidget(lblSubRegionIndex_, 1, 0);
    layout_->addWidget(btnSubRegionPPrev_, 1, 1);
    layout_->addWidget(btnSubRegionPrev_,  1, 2, 1, 3);
    layout_->addWidget(btnSubRegionNext_,  1, 5, 1, 3);
    layout_->addWidget(btnSubRegionNNext_, 1, 8);

    layout_->addWidget(optionsFrame_, 2, 0, 1, 9);
    layout_->addWidget(debugMessage_, 3, 0, 1, 9);
    layout_->addWidget(rawRegion_, 4, 0, 1, 9);
    layout_->addWidget(editedRegion_, 5, 0, 1, 9);

    updateFrame(app_status, NULL, NULL);
}

void DebugFrame::drawOptionControls(AppStatus &app_status) {
    if (optionsAreSame(app_status)) {
        // Nothing has changed, don't redraw anything
        return;
    }

    clearOptionControls();

    VatnaoOptionEntry *opt;
    int i = 0;
    for (std::vector<VatnaoOption>::iterator it = app_status.options.begin();
         it != app_status.options.end(); it++, i++) {

        opt = new VatnaoOptionEntry(vatnao_manager_, *it);
        options_.push_back(opt);
        optionsLayout_->addWidget(opt, i, 0);
    }
}

bool DebugFrame::optionsAreSame(AppStatus &app_status) {
    if (app_status.options.size() != options_.size()) {
        return false;
    }

    for (size_t i = 0; i < app_status.options.size(); i++) {
        VatnaoOption a = app_status.options[i];
        VatnaoOption b = options_[i]->getVatnaoOption();
        if (a != b) {
            return false;
        }
    }
    return true;
}

void DebugFrame::clearOptionControls() {
    for (std::vector<VatnaoOptionEntry*>::iterator it = options_.begin();
         it != options_.end(); it++) {

        delete &(*it);
    }
    options_.clear();
}


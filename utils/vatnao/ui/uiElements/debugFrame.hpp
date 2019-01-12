#ifndef VATNAO_UI_UIELEMENTS_DEBUG_FRAME_HPP
#define VATNAO_UI_UIELEMENTS_DEBUG_FRAME_HPP

#include <QtGui>
#include <QPushButton>
#include <QObject>
#include <QGridLayout>
#include <QScrollArea>

#include "../vatnaoManager.hpp"
#include "option.hpp"

#include "imageView.hpp"

class DebugFrame: public QFrame {

    Q_OBJECT
    public:
    DebugFrame(VatnaoManager& vatnao_manager, QWidget *parent = 0);
    ~DebugFrame();

    public slots:
    void updateFrame(AppStatus& app_status,
                     RgbImg* previewRegion,
                     RgbImg* annotatedRegion);
    private:
    QWidget *parent_;
    VatnaoManager& vatnao_manager_;

    void initControls(AppStatus& app_status);
    void drawOptionControls(AppStatus& app_status);
    bool optionsAreSame(AppStatus& app_status);
    void clearOptionControls();

    ImageView *rawRegion_;
    ImageView *editedRegion_;

    QPushButton *btnRegionPPrev_;
    QPushButton *btnRegionPrev_;
    QPushButton *btnRegionNext_;
    QPushButton *btnRegionNNext_;
    QLabel      *lblRegionIndex_;

    QPushButton *btnSubRegionPPrev_;
    QPushButton *btnSubRegionPrev_;
    QPushButton *btnSubRegionNext_;
    QPushButton *btnSubRegionNNext_;
    QLabel      *lblSubRegionIndex_;

    QFrame      *optionsFrame_;
    QGridLayout *optionsLayout_;

    QTextEdit   *debugMessage_;

    std::vector<VatnaoOptionEntry*> options_;

    QGridLayout *layout_;
};

#endif

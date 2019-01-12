#ifndef VATNAO_UI_UIELEMENTS_OPTION_HPP
#define VATNAO_UI_UIELEMENTS_OPTION_HPP

#include <string>

#include <QLabel>
#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGridLayout>

#include "../vatnaoManager.hpp"
#include "../../app/appStatus.hpp"

class VatnaoOptionEntry: public QFrame {
    Q_OBJECT
    public:
        VatnaoOptionEntry(VatnaoManager &vatnao_manager_, VatnaoOption option, QWidget *parent = 0);
        ~VatnaoOptionEntry();

        std::string getName();
        std::string getCurrOption();
        double getCurrNumeric();
        bool isSelected();

        VatnaoOption getVatnaoOption();
    private slots:
        void valueChanged();
        void valueChanged(const QString& str);
    private:
        VatnaoManager &vatnao_manager_;
        VatnaoOption option_;

        QLabel         *label_;
        QComboBox      *dropdown_;
        QCheckBox      *checkbox_;
        QDoubleSpinBox *numeric_box_;

        QGridLayout *layout_;

};

#endif

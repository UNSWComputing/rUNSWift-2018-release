#include "option.hpp"

#include <QtGui>
#include <QObject>
#include <QString>

VatnaoOptionEntry::VatnaoOptionEntry(VatnaoManager &vatnao_manager_, VatnaoOption option, QWidget *parent):
    QFrame(parent),
    vatnao_manager_(vatnao_manager_)
{
    checkbox_ = NULL;
    dropdown_ = NULL;
    numeric_box_ = NULL;

    option_ = option;
    label_ = new QLabel();
    label_->setText(QString(option_.option_name.c_str()));

    layout_ = new QGridLayout(this);
    layout_->addWidget(label_, 0, 0);

    if (option_.is_boolean_option()) {
        checkbox_ = new QCheckBox;

        QObject::connect(checkbox_, SIGNAL(clicked()),
                         this, SLOT(valueChanged()));
        layout_->addWidget(checkbox_, 0, 1);
    } else if (option_.is_numeric) {
        numeric_box_ = new QDoubleSpinBox;
        numeric_box_->setDecimals(8);

        QObject::connect(numeric_box_, SIGNAL(valueChanged(const QString&)),
                         this, SLOT(valueChanged(const QString&)));
        layout_->addWidget(numeric_box_, 0, 1);
    } else {
        dropdown_ = new QComboBox;
        for (std::vector<std::string>::iterator it = option_.options.begin();
             it != option_.options.end();
             ++it) {
            dropdown_->addItem(QString(it->c_str()));
        }

        QObject::connect(dropdown_, SIGNAL(currentIndexChanged(const QString&)),
                         this, SLOT(valueChanged(const QString&)));
        layout_->addWidget(dropdown_, 0, 1);
    }

}

VatnaoOptionEntry::~VatnaoOptionEntry() {
    delete label_;
    delete layout_;
    if (checkbox_ != NULL) {
        delete checkbox_;
    }
    if (dropdown_ != NULL) {
        delete dropdown_;
    }
    if (numeric_box_ != NULL) {
        delete numeric_box_;
    }
}

void VatnaoOptionEntry::valueChanged() {
    if (option_.is_boolean_option()) {
        vatnao_manager_.updateBooleanOption(option_.option_name, isSelected());
    } else if (option_.is_numeric) {
        vatnao_manager_.updateNumericOption(option_.option_name, getCurrNumeric());
    } else {
        vatnao_manager_.updateStringOption(option_.option_name, getCurrOption());
    }
    vatnao_manager_.reprocess();
}

void VatnaoOptionEntry::valueChanged(const QString& str) {
    valueChanged();
}

std::string VatnaoOptionEntry::getName() {
    return option_.option_name;
}

std::string VatnaoOptionEntry::getCurrOption() {
    if (!option_.is_boolean_option() && !option_.is_numeric) {
        return dropdown_->currentText().toUtf8().constData();
    }
    // If not a dropdown, just always return blank
    return "";
}

double VatnaoOptionEntry::getCurrNumeric() {
    if (option_.is_numeric) {
        return numeric_box_->value();
    }
    // If not a numeric selector, just return 0
    return 0;
}

bool VatnaoOptionEntry::isSelected() {
    if (option_.is_boolean_option()) {
        return checkbox_->isChecked();
    }
    // If not a checkbox, just always return false
    return false;
}

VatnaoOption VatnaoOptionEntry::getVatnaoOption() {
    return option_;
}

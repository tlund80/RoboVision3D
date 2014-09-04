/********************************************************************************
** Form generated from reading UI file 'AnnotateDialog.ui'
**
** Created: Mon Mar 11 20:19:55 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ANNOTATEDIALOG_H
#define UI_ANNOTATEDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_annotationDialog
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *_imgLabel;
    QHBoxLayout *horizontalLayout;
    QPushButton *btnOk;
    QPushButton *btnSkip;

    void setupUi(QDialog *annotationDialog)
    {
        if (annotationDialog->objectName().isEmpty())
            annotationDialog->setObjectName(QString::fromUtf8("annotationDialog"));
        annotationDialog->resize(400, 300);
        annotationDialog->setModal(false);
        verticalLayout = new QVBoxLayout(annotationDialog);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        _imgLabel = new QLabel(annotationDialog);
        _imgLabel->setObjectName(QString::fromUtf8("_imgLabel"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(_imgLabel->sizePolicy().hasHeightForWidth());
        _imgLabel->setSizePolicy(sizePolicy);
        _imgLabel->setContextMenuPolicy(Qt::NoContextMenu);
        _imgLabel->setFrameShape(QFrame::NoFrame);

        verticalLayout->addWidget(_imgLabel);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        btnOk = new QPushButton(annotationDialog);
        btnOk->setObjectName(QString::fromUtf8("btnOk"));

        horizontalLayout->addWidget(btnOk);

        btnSkip = new QPushButton(annotationDialog);
        btnSkip->setObjectName(QString::fromUtf8("btnSkip"));

        horizontalLayout->addWidget(btnSkip);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(annotationDialog);
        QObject::connect(btnSkip, SIGNAL(released()), annotationDialog, SLOT(close()));

        QMetaObject::connectSlotsByName(annotationDialog);
    } // setupUi

    void retranslateUi(QDialog *annotationDialog)
    {
        annotationDialog->setWindowTitle(QApplication::translate("annotationDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        _imgLabel->setText(QString());
        btnOk->setText(QApplication::translate("annotationDialog", "Apply Roi", 0, QApplication::UnicodeUTF8));
        btnSkip->setText(QApplication::translate("annotationDialog", "Skip image", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class annotationDialog: public Ui_annotationDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ANNOTATEDIALOG_H

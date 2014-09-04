/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created: Fri May 23 19:14:29 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFormLayout>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QSplitter>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QTextBrowser>
#include <QtGui/QToolButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_7;
    QGroupBox *verticalGroupBox;
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget_2;
    QWidget *tabCameraCalib;
    QGridLayout *gridLayout_3;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_3;
    QLabel *label;
    QComboBox *comboSelectCalibType;
    QLabel *labelPreviewSize;
    QHBoxLayout *horizontalLayout_9;
    QComboBox *comboPreviewSize;
    QLineEdit *lineEditPreviewWidth;
    QLineEdit *lineEditPreviewHeight;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout_6;
    QLabel *_fileFilterL_label;
    QLineEdit *lineEditNameFilterL;
    QLineEdit *imgPathL;
    QHBoxLayout *horizontalLayout_5;
    QToolButton *btnFileBrowseL;
    QHBoxLayout *horizontalLayout_8;
    QLabel *_fileFilterR_label;
    QLineEdit *lineEditNameFilterR;
    QLineEdit *imgPathR;
    QHBoxLayout *horizontalLayout_4;
    QToolButton *btnFileBrowseR;
    QGroupBox *groupBox_3;
    QFormLayout *formLayout;
    QLabel *label_5;
    QComboBox *comboBoardType;
    QLabel *label_6;
    QHBoxLayout *horizontalLayout_10;
    QSpinBox *spinBoxBoardSizeX;
    QSpinBox *spinBoxBoardSizeY;
    QLabel *label_7;
    QHBoxLayout *horizontalLayout_11;
    QDoubleSpinBox *_squarePhysicalSize_SPINB;
    QTabWidget *tabWidget;
    QWidget *tabCornerFlags;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label_8;
    QComboBox *_annotationType_Combo;
    QCheckBox *_previewCorner_CB;
    QCheckBox *_normalizeImg_CB;
    QCheckBox *_adaptiveThres_CB;
    QCheckBox *_preCornerAnalysis_CB;
    QCheckBox *_filterQuads_CB;
    QGroupBox *_itc_IterativeTreminationFlags_Corner_BOX;
    QVBoxLayout *verticalLayout_5;
    QLabel *label_9;
    QHBoxLayout *horizontalLayout_13;
    QCheckBox *_itc_iterations_Corner_checkBox;
    QSpacerItem *horizontalSpacer;
    QLineEdit *_itc_max_Iter_Corner_LineEdit_;
    QHBoxLayout *horizontalLayout_14;
    QCheckBox *_itc_epsilon_Corner_checkBox_2;
    QSpacerItem *horizontalSpacer_2;
    QLineEdit *_itc_epsilon_Corner_LineEdit;
    QSpacerItem *verticalSpacer_3;
    QWidget *tabCalFlags;
    QVBoxLayout *verticalLayout_8;
    QGroupBox *_CalFlags_groupBox;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_16;
    QCheckBox *_useRationalModel_CB;
    QCheckBox *_useIntrinsicGuess_CB;
    QHBoxLayout *_Cam1Intrinsicload_container;
    QLabel *_intrinsicGuessCam1label;
    QLineEdit *_intrinsicGuessCam1Path;
    QToolButton *_intrinsicGuessCam1btnFileBrowse;
    QHBoxLayout *horizontalLayout_15;
    QLabel *_intrinsicGuessCam2label;
    QLineEdit *_intrinsicGuessCam2Path;
    QToolButton *_intrinsicGuessCam2btnFileBrowse;
    QGroupBox *groupBox_inspectIntrinsic;
    QVBoxLayout *verticalLayout_9;
    QTabWidget *_inspectCamTabs;
    QWidget *_inspectIntrinsicCamera1Tab;
    QVBoxLayout *verticalLayout_10;
    QHBoxLayout *horizontalLayout_17;
    QPushButton *_intrinsicReadFromFileBtnL;
    QPushButton *_intrinsicClearValsBtnL;
    QCheckBox *_fixPrincipalPoint_CB_CL;
    QHBoxLayout *horizontalLayout_19;
    QLabel *label_12;
    QLineEdit *_fixPrincipalPoint_VALx_CL;
    QHBoxLayout *horizontalLayout_20;
    QLabel *label_13;
    QLineEdit *_fixPrincipalPoint_VALy_CL;
    QHBoxLayout *horizontalLayout_23;
    QCheckBox *_fixFocalLength_CB_CL;
    QCheckBox *_fixAspectRatio_CB_CL;
    QHBoxLayout *horizontalLayout_26;
    QLabel *label_16;
    QLineEdit *_fixFocalLength_VALx_CL;
    QHBoxLayout *horizontalLayout_27;
    QLabel *label_17;
    QLineEdit *_fixFocalLength_VALy_CL;
    QLabel *label_20;
    QCheckBox *_fixZeroTangentDistortion_CB_CL;
    QHBoxLayout *horizontalLayout_31;
    QLabel *label_24;
    QLineEdit *_fix_P1_VAL_CL;
    QHBoxLayout *horizontalLayout_32;
    QLabel *label_25;
    QLineEdit *_fix_P2_VAL_CL;
    QHBoxLayout *horizontalLayout_33;
    QCheckBox *_fix_K1_CB_CL;
    QLineEdit *_fix_K1_VAL_CL;
    QHBoxLayout *horizontalLayout_34;
    QCheckBox *_fix_K2_CB_CL;
    QLineEdit *_fix_K2_VAL_CL;
    QHBoxLayout *horizontalLayout_35;
    QCheckBox *_fix_K3_CB_CL;
    QLineEdit *_fix_K3_VAL_CL;
    QHBoxLayout *horizontalLayout_36;
    QCheckBox *_fix_K4_CB_CL;
    QLineEdit *_fix_K4_VAL_CL;
    QHBoxLayout *horizontalLayout_37;
    QCheckBox *_fix_K5_CB_CL;
    QLineEdit *_fix_K5_VAL_CL;
    QHBoxLayout *horizontalLayout_38;
    QCheckBox *_fix_K6_CB_CL;
    QLineEdit *_fix_K6_VAL_CL;
    QWidget *_inspectIntrinsicCamera2Tab;
    QVBoxLayout *verticalLayout_11;
    QHBoxLayout *horizontalLayout_18;
    QPushButton *_intrinsicReadFromFileBtnR;
    QPushButton *_intrinsicClearValsBtnR;
    QCheckBox *_fixPrincipalPoint_CB_CR;
    QHBoxLayout *horizontalLayout_21;
    QLabel *label_14;
    QLineEdit *_fixPrincipalPoint_VALx_CR;
    QHBoxLayout *horizontalLayout_22;
    QLabel *label_15;
    QLineEdit *_fixPrincipalPoint_VALy_CR;
    QHBoxLayout *horizontalLayout_24;
    QCheckBox *_fixFocalLength_CB_CR;
    QCheckBox *_fixAspectRatio_CB_CR;
    QHBoxLayout *horizontalLayout_25;
    QLabel *label_18;
    QLineEdit *_fixFocalLength_VALx_CR;
    QHBoxLayout *horizontalLayout_28;
    QLabel *label_19;
    QLineEdit *_fixFocalLength_VALy_CR;
    QLabel *label_21;
    QCheckBox *_fixZeroTangentDistortion_CB_CR;
    QHBoxLayout *horizontalLayout_29;
    QLabel *label_22;
    QLineEdit *_fix_P1_VAL_CR;
    QHBoxLayout *horizontalLayout_30;
    QLabel *label_23;
    QLineEdit *_fix_P2_VAL_CR;
    QHBoxLayout *horizontalLayout_39;
    QCheckBox *_fix_K1_CB_CR;
    QLineEdit *_fix_K1_VAL_CR;
    QHBoxLayout *horizontalLayout_40;
    QCheckBox *_fix_K2_CB_CR;
    QLineEdit *_fix_K2_VAL_CR;
    QHBoxLayout *horizontalLayout_41;
    QCheckBox *_fix_K3_CB_CR;
    QLineEdit *_fix_K3_VAL_CR;
    QHBoxLayout *horizontalLayout_42;
    QCheckBox *_fix_K4_CB_CR;
    QLineEdit *_fix_K4_VAL_CR;
    QHBoxLayout *horizontalLayout_43;
    QCheckBox *_fix_K5_CB_CR;
    QLineEdit *_fix_K5_VAL_CR;
    QHBoxLayout *horizontalLayout_44;
    QCheckBox *_fix_K6_CB_CR;
    QLineEdit *_fix_K6_VAL_CR;
    QGroupBox *groupBox_StereoCalFlags;
    QVBoxLayout *verticalLayout_6;
    QCheckBox *_fixIntrinsicStereo_CB;
    QCheckBox *_sameFocalLengthStereo_CB;
    QGroupBox *_itc_IterativeTreminationFlags_Stereo_BOX;
    QVBoxLayout *verticalLayout_12;
    QLabel *label_26;
    QHBoxLayout *horizontalLayout_45;
    QCheckBox *_itc_iterations_Stereo_checkBox;
    QSpacerItem *horizontalSpacer_3;
    QLineEdit *_itc_max_Iter_Stereo_LineEdit;
    QHBoxLayout *horizontalLayout_46;
    QCheckBox *_itc_epsilon_Stereo_checkBox;
    QSpacerItem *horizontalSpacer_4;
    QLineEdit *_itc_epsilon_Stereo_LineEdit;
    QSpacerItem *verticalSpacer;
    QWidget *tabRectFlags;
    QVBoxLayout *verticalLayout_13;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_14;
    QHBoxLayout *horizontalLayout_47;
    QLabel *_rectAlgorithm_Label;
    QSpacerItem *horizontalSpacer_5;
    QComboBox *_rectAlgorithm_Combo;
    QHBoxLayout *horizontalLayout_48;
    QLabel *_rectFundamentalLabel;
    QSpacerItem *horizontalSpacer_6;
    QComboBox *_rectFundamental_Combo;
    QHBoxLayout *horizontalLayout_49;
    QLabel *_rectParam1_label;
    QLineEdit *_rectParam1_VAL;
    QLabel *_rectParam2_label;
    QLineEdit *_rectParam2_VAL;
    QHBoxLayout *horizontalLayout_50;
    QCheckBox *_rectAlpha_CB;
    QSpacerItem *horizontalSpacer_7;
    QDoubleSpinBox *_rectAlpha_VAL;
    QCheckBox *_zeroRectifyDisparity_CB;
    QCheckBox *_rectDisplayValidRect_CB;
    QCheckBox *_displayRectification_CB;
    QCheckBox *_saveRectification_CB;
    QCheckBox *_rectAllImgsInFolder;
    QHBoxLayout *horizontalLayout_51;
    QLabel *_rectSavePathLeftLabel;
    QSpacerItem *horizontalSpacer_8;
    QLineEdit *_rectificationSavePathTxtL;
    QToolButton *_rectificationSavePathBtnL;
    QHBoxLayout *horizontalLayout_52;
    QLabel *_rectSavePathRightLabel;
    QSpacerItem *horizontalSpacer_9;
    QLineEdit *_rectificationSavePathTxtR;
    QToolButton *_rectificationSavePathBtnR;
    QSpacerItem *verticalSpacer_2;
    QWidget *tabNarrowStereo;
    QWidget *layoutWidget;
    QGridLayout *gridLayout_2;
    QLabel *label_10;
    QLineEdit *lineEditNarrowIntrinsicLeft;
    QToolButton *btnFileBrowserNSIntrinsicLeft;
    QLabel *label_11;
    QLineEdit *lineEditNarrowIntrinsicRight;
    QToolButton *btnFileBrowserNSIntrinsicRight;
    QLabel *label_27;
    QLineEdit *lineEditNarrowImageLeft;
    QToolButton *btnFileBrowserNSLeftImage;
    QLabel *label_28;
    QLineEdit *lineEditNarrowImageRight;
    QToolButton *btnFileBrowserNSRightImage;
    QLabel *label_30;
    QLineEdit *lineEditNarrowOutputLeft;
    QLabel *label_31;
    QLineEdit *lineEditNarrowOutputRight;
    QPushButton *pushButtonNarrowCompute;
    QVBoxLayout *verticalLayout_28;
    QHBoxLayout *horizontalLayout_103;
    QSpacerItem *verticalSpacer_4;
    QVBoxLayout *verticalLayout_16;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *_doSaveResultsBtn;
    QPushButton *_clearAllData;
    QPushButton *_commitROS;
    QPushButton *_cancelBtn;
    QHBoxLayout *horizontalLayout_105;
    QPushButton *_doDetectCornersBtn;
    QPushButton *_doCalibrateBtn;
    QPushButton *_doRectifyBtn;
    QPushButton *_doAllBtn;
    QWidget *tabHandEye;
    QGridLayout *gridLayout_5;
    QScrollArea *scrollArea_2;
    QWidget *scrollAreaWidgetContents_2;
    QVBoxLayout *verticalLayout_15;
    QLabel *label_29;
    QComboBox *comboSelectCalibType_2;
    QLabel *labelPreviewSize_2;
    QHBoxLayout *horizontalLayout;
    QComboBox *comboPreviewSize_2;
    QLineEdit *lineEditPreviewWidth_2;
    QLineEdit *lineEditPreviewHeight_2;
    QLabel *label_32;
    QHBoxLayout *horizontalLayout_3;
    QLabel *_fileFilterL_label_2;
    QLineEdit *lineEditNameFilterL_2;
    QLineEdit *imgPathL_2;
    QToolButton *btnFileBrowseL_2;
    QGroupBox *groupBox_4;
    QFormLayout *formLayout_2;
    QLabel *label_33;
    QComboBox *comboBoardType_2;
    QLabel *label_34;
    QHBoxLayout *horizontalLayout_53;
    QSpinBox *spinBoxBoardSizeX_2;
    QSpinBox *spinBoxBoardSizeY_2;
    QLabel *label_35;
    QHBoxLayout *horizontalLayout_54;
    QDoubleSpinBox *_squarePhysicalSize_SPINB_2;
    QLabel *labelPreviewSize_3;
    QHBoxLayout *_Cam1Intrinsicload_container_2;
    QLabel *_intrinsicGuessCam1label_2;
    QLineEdit *_intrinsicGuessCam1Path_2;
    QToolButton *_intrinsicGuessCam1btnFileBrowse_2;
    QTabWidget *tabWidget_3;
    QWidget *tab;
    QGridLayout *gridLayout_6;
    QGroupBox *groupBoxDetectCorners;
    QGridLayout *gridLayout_4;
    QSplitter *splitter;
    QLabel *label_38;
    QComboBox *_annotationType_Combo_3;
    QCheckBox *_previewCorner_CB_2;
    QCheckBox *_normalizeImg_CB_2;
    QCheckBox *_adaptiveThres_CB_2;
    QCheckBox *_preCornerAnalysis_CB_2;
    QCheckBox *_filterQuads_CB_2;
    QGroupBox *_itc_IterativeTreminationFlags_Corner_BOX_2;
    QVBoxLayout *verticalLayout_17;
    QLabel *label_39;
    QHBoxLayout *horizontalLayout_61;
    QCheckBox *_itc_iterations_Corner_checkBox_3;
    QSpacerItem *horizontalSpacer_12;
    QLineEdit *_itc_max_Iter_Corner_LineEdit_2;
    QHBoxLayout *horizontalLayout_62;
    QCheckBox *_itc_epsilon_Corner_checkBox_4;
    QSpacerItem *horizontalSpacer_13;
    QLineEdit *_itc_epsilon_Corner_LineEdit_3;
    QSpacerItem *verticalSpacer_6;
    QWidget *tab_2;
    QGroupBox *groupBoxRobotPoseOnline;
    QCheckBox *checkSaveLiveRobotPoses;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_58;
    QLabel *label_36;
    QComboBox *comboAngleType;
    QLabel *_RobotPosePathlabel_2;
    QWidget *layoutWidget_6;
    QHBoxLayout *_Cam1Intrinsicload_container_5;
    QLineEdit *_RobotPoseSavePath;
    QToolButton *_RobotSavePathbtnFileBrowse_3;
    QCheckBox *checkDisplayLiveRobotPoses;
    QLabel *label_37;
    QLabel *labelRobotPoseCount;
    QWidget *layoutWidget2;
    QHBoxLayout *horizontalLayout_60;
    QPushButton *_btnSaveRobotPoses;
    QPushButton *_btnClearRobotPoses;
    QGroupBox *groupBoxLoadRobotData;
    QWidget *layoutWidget_5;
    QHBoxLayout *_Cam1Intrinsicload_container_3;
    QLineEdit *_RobotPosePath;
    QToolButton *_RobotPathbtnFileBrowse;
    QPushButton *_intrinsicReadFromFileBtnR_2;
    QHBoxLayout *horizontalLayout_59;
    QPushButton *_doSaveResultsBtn_2;
    QPushButton *_clearAllData_2;
    QPushButton *_commitROS_2;
    QPushButton *_cancelBtn_2;
    QHBoxLayout *horizontalLayout_56;
    QPushButton *_doDetectCornersBtn_2;
    QPushButton *_doComputeCamPoseBtn_2;
    QPushButton *_doCalibrateBtn_2;
    QPushButton *_doAllBtn_2;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_imgPreview;
    QVBoxLayout *verticalLayout_previewL;
    QLabel *imgLabelL;
    QVBoxLayout *verticalLayout_previewR;
    QLabel *imgLabelR;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout_55;
    QLabel *label_4;
    QComboBox *comboBoxImageCurrent;
    QPushButton *pushButtonImageDelete;
    QGroupBox *liveStreamBox;
    QGridLayout *gridLayout;
    QLineEdit *lineEditImgTopicR;
    QPushButton *pushButtonStreamSaveImage;
    QPushButton *pushButtonStreamStop;
    QPushButton *pushButtonStreamStart;
    QLabel *label_2;
    QLineEdit *lineEditImgTopicL;
    QHBoxLayout *horizontalLayout_57;
    QLabel *labelRightTopic;
    QCheckBox *checkBoxIsInverted;
    QLabel *labelLeftTopic;
    QTextBrowser *_console;
    QPushButton *_clearConsoleBtn;
    QMenuBar *menubar;
    QMenu *menu_File;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(1180, 800);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(100);
        sizePolicy.setVerticalStretch(100);
        sizePolicy.setHeightForWidth(MainWindowDesign->sizePolicy().hasHeightForWidth());
        MainWindowDesign->setSizePolicy(sizePolicy);
        MainWindowDesign->setMinimumSize(QSize(250, 0));
        MainWindowDesign->setMaximumSize(QSize(1600, 1200));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QString::fromUtf8("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QString::fromUtf8("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout_7 = new QHBoxLayout(centralwidget);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        verticalGroupBox = new QGroupBox(centralwidget);
        verticalGroupBox->setObjectName(QString::fromUtf8("verticalGroupBox"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::MinimumExpanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(verticalGroupBox->sizePolicy().hasHeightForWidth());
        verticalGroupBox->setSizePolicy(sizePolicy1);
        verticalGroupBox->setMinimumSize(QSize(530, 0));
        verticalGroupBox->setMaximumSize(QSize(500, 16777215));
        verticalLayout = new QVBoxLayout(verticalGroupBox);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        tabWidget_2 = new QTabWidget(verticalGroupBox);
        tabWidget_2->setObjectName(QString::fromUtf8("tabWidget_2"));
        tabWidget_2->setEnabled(true);
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
        sizePolicy2.setHorizontalStretch(100);
        sizePolicy2.setVerticalStretch(100);
        sizePolicy2.setHeightForWidth(tabWidget_2->sizePolicy().hasHeightForWidth());
        tabWidget_2->setSizePolicy(sizePolicy2);
        tabWidget_2->setMinimumSize(QSize(400, 0));
        tabCameraCalib = new QWidget();
        tabCameraCalib->setObjectName(QString::fromUtf8("tabCameraCalib"));
        gridLayout_3 = new QGridLayout(tabCameraCalib);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        scrollArea = new QScrollArea(tabCameraCalib);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        QSizePolicy sizePolicy3(QSizePolicy::MinimumExpanding, QSizePolicy::Expanding);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(100);
        sizePolicy3.setHeightForWidth(scrollArea->sizePolicy().hasHeightForWidth());
        scrollArea->setSizePolicy(sizePolicy3);
        scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 464, 1431));
        verticalLayout_3 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        label = new QLabel(scrollAreaWidgetContents);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_3->addWidget(label);

        comboSelectCalibType = new QComboBox(scrollAreaWidgetContents);
        comboSelectCalibType->setObjectName(QString::fromUtf8("comboSelectCalibType"));

        verticalLayout_3->addWidget(comboSelectCalibType);

        labelPreviewSize = new QLabel(scrollAreaWidgetContents);
        labelPreviewSize->setObjectName(QString::fromUtf8("labelPreviewSize"));

        verticalLayout_3->addWidget(labelPreviewSize);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        comboPreviewSize = new QComboBox(scrollAreaWidgetContents);
        comboPreviewSize->setObjectName(QString::fromUtf8("comboPreviewSize"));
        QSizePolicy sizePolicy4(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(comboPreviewSize->sizePolicy().hasHeightForWidth());
        comboPreviewSize->setSizePolicy(sizePolicy4);
        comboPreviewSize->setMinimumSize(QSize(100, 20));
        comboPreviewSize->setMaximumSize(QSize(2000, 20));

        horizontalLayout_9->addWidget(comboPreviewSize);

        lineEditPreviewWidth = new QLineEdit(scrollAreaWidgetContents);
        lineEditPreviewWidth->setObjectName(QString::fromUtf8("lineEditPreviewWidth"));

        horizontalLayout_9->addWidget(lineEditPreviewWidth);

        lineEditPreviewHeight = new QLineEdit(scrollAreaWidgetContents);
        lineEditPreviewHeight->setObjectName(QString::fromUtf8("lineEditPreviewHeight"));

        horizontalLayout_9->addWidget(lineEditPreviewHeight);


        verticalLayout_3->addLayout(horizontalLayout_9);

        label_3 = new QLabel(scrollAreaWidgetContents);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_3->addWidget(label_3);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        _fileFilterL_label = new QLabel(scrollAreaWidgetContents);
        _fileFilterL_label->setObjectName(QString::fromUtf8("_fileFilterL_label"));
        _fileFilterL_label->setMinimumSize(QSize(37, 0));

        horizontalLayout_6->addWidget(_fileFilterL_label);

        lineEditNameFilterL = new QLineEdit(scrollAreaWidgetContents);
        lineEditNameFilterL->setObjectName(QString::fromUtf8("lineEditNameFilterL"));
        QSizePolicy sizePolicy5(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy5.setHorizontalStretch(0);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(lineEditNameFilterL->sizePolicy().hasHeightForWidth());
        lineEditNameFilterL->setSizePolicy(sizePolicy5);

        horizontalLayout_6->addWidget(lineEditNameFilterL);

        imgPathL = new QLineEdit(scrollAreaWidgetContents);
        imgPathL->setObjectName(QString::fromUtf8("imgPathL"));

        horizontalLayout_6->addWidget(imgPathL);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        btnFileBrowseL = new QToolButton(scrollAreaWidgetContents);
        btnFileBrowseL->setObjectName(QString::fromUtf8("btnFileBrowseL"));

        horizontalLayout_5->addWidget(btnFileBrowseL);


        horizontalLayout_6->addLayout(horizontalLayout_5);


        verticalLayout_3->addLayout(horizontalLayout_6);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        _fileFilterR_label = new QLabel(scrollAreaWidgetContents);
        _fileFilterR_label->setObjectName(QString::fromUtf8("_fileFilterR_label"));

        horizontalLayout_8->addWidget(_fileFilterR_label);

        lineEditNameFilterR = new QLineEdit(scrollAreaWidgetContents);
        lineEditNameFilterR->setObjectName(QString::fromUtf8("lineEditNameFilterR"));
        sizePolicy5.setHeightForWidth(lineEditNameFilterR->sizePolicy().hasHeightForWidth());
        lineEditNameFilterR->setSizePolicy(sizePolicy5);

        horizontalLayout_8->addWidget(lineEditNameFilterR);

        imgPathR = new QLineEdit(scrollAreaWidgetContents);
        imgPathR->setObjectName(QString::fromUtf8("imgPathR"));

        horizontalLayout_8->addWidget(imgPathR);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        btnFileBrowseR = new QToolButton(scrollAreaWidgetContents);
        btnFileBrowseR->setObjectName(QString::fromUtf8("btnFileBrowseR"));

        horizontalLayout_4->addWidget(btnFileBrowseR);


        horizontalLayout_8->addLayout(horizontalLayout_4);


        verticalLayout_3->addLayout(horizontalLayout_8);

        groupBox_3 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        formLayout = new QFormLayout(groupBox_3);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        label_5 = new QLabel(groupBox_3);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_5);

        comboBoardType = new QComboBox(groupBox_3);
        comboBoardType->setObjectName(QString::fromUtf8("comboBoardType"));

        formLayout->setWidget(0, QFormLayout::FieldRole, comboBoardType);

        label_6 = new QLabel(groupBox_3);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_6);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        spinBoxBoardSizeX = new QSpinBox(groupBox_3);
        spinBoxBoardSizeX->setObjectName(QString::fromUtf8("spinBoxBoardSizeX"));
        spinBoxBoardSizeX->setValue(10);

        horizontalLayout_10->addWidget(spinBoxBoardSizeX);

        spinBoxBoardSizeY = new QSpinBox(groupBox_3);
        spinBoxBoardSizeY->setObjectName(QString::fromUtf8("spinBoxBoardSizeY"));
        spinBoxBoardSizeY->setValue(6);

        horizontalLayout_10->addWidget(spinBoxBoardSizeY);


        formLayout->setLayout(1, QFormLayout::FieldRole, horizontalLayout_10);

        label_7 = new QLabel(groupBox_3);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_7);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        _squarePhysicalSize_SPINB = new QDoubleSpinBox(groupBox_3);
        _squarePhysicalSize_SPINB->setObjectName(QString::fromUtf8("_squarePhysicalSize_SPINB"));
        _squarePhysicalSize_SPINB->setDecimals(5);
        _squarePhysicalSize_SPINB->setMaximum(1e+09);
        _squarePhysicalSize_SPINB->setValue(100);

        horizontalLayout_11->addWidget(_squarePhysicalSize_SPINB);


        formLayout->setLayout(2, QFormLayout::FieldRole, horizontalLayout_11);


        verticalLayout_3->addWidget(groupBox_3);

        tabWidget = new QTabWidget(scrollAreaWidgetContents);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        QSizePolicy sizePolicy6(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy6.setHorizontalStretch(0);
        sizePolicy6.setVerticalStretch(0);
        sizePolicy6.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy6);
        tabCornerFlags = new QWidget();
        tabCornerFlags->setObjectName(QString::fromUtf8("tabCornerFlags"));
        verticalLayout_4 = new QVBoxLayout(tabCornerFlags);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        label_8 = new QLabel(tabCornerFlags);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_12->addWidget(label_8);

        _annotationType_Combo = new QComboBox(tabCornerFlags);
        _annotationType_Combo->setObjectName(QString::fromUtf8("_annotationType_Combo"));

        horizontalLayout_12->addWidget(_annotationType_Combo);


        verticalLayout_4->addLayout(horizontalLayout_12);

        _previewCorner_CB = new QCheckBox(tabCornerFlags);
        _previewCorner_CB->setObjectName(QString::fromUtf8("_previewCorner_CB"));
        _previewCorner_CB->setChecked(true);

        verticalLayout_4->addWidget(_previewCorner_CB);

        _normalizeImg_CB = new QCheckBox(tabCornerFlags);
        _normalizeImg_CB->setObjectName(QString::fromUtf8("_normalizeImg_CB"));
        _normalizeImg_CB->setChecked(true);

        verticalLayout_4->addWidget(_normalizeImg_CB);

        _adaptiveThres_CB = new QCheckBox(tabCornerFlags);
        _adaptiveThres_CB->setObjectName(QString::fromUtf8("_adaptiveThres_CB"));
        _adaptiveThres_CB->setChecked(true);

        verticalLayout_4->addWidget(_adaptiveThres_CB);

        _preCornerAnalysis_CB = new QCheckBox(tabCornerFlags);
        _preCornerAnalysis_CB->setObjectName(QString::fromUtf8("_preCornerAnalysis_CB"));
        _preCornerAnalysis_CB->setChecked(true);

        verticalLayout_4->addWidget(_preCornerAnalysis_CB);

        _filterQuads_CB = new QCheckBox(tabCornerFlags);
        _filterQuads_CB->setObjectName(QString::fromUtf8("_filterQuads_CB"));

        verticalLayout_4->addWidget(_filterQuads_CB);

        _itc_IterativeTreminationFlags_Corner_BOX = new QGroupBox(tabCornerFlags);
        _itc_IterativeTreminationFlags_Corner_BOX->setObjectName(QString::fromUtf8("_itc_IterativeTreminationFlags_Corner_BOX"));
        _itc_IterativeTreminationFlags_Corner_BOX->setCheckable(true);
        verticalLayout_5 = new QVBoxLayout(_itc_IterativeTreminationFlags_Corner_BOX);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        label_9 = new QLabel(_itc_IterativeTreminationFlags_Corner_BOX);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        verticalLayout_5->addWidget(label_9);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        _itc_iterations_Corner_checkBox = new QCheckBox(_itc_IterativeTreminationFlags_Corner_BOX);
        _itc_iterations_Corner_checkBox->setObjectName(QString::fromUtf8("_itc_iterations_Corner_checkBox"));
        _itc_iterations_Corner_checkBox->setChecked(true);

        horizontalLayout_13->addWidget(_itc_iterations_Corner_checkBox);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_13->addItem(horizontalSpacer);

        _itc_max_Iter_Corner_LineEdit_ = new QLineEdit(_itc_IterativeTreminationFlags_Corner_BOX);
        _itc_max_Iter_Corner_LineEdit_->setObjectName(QString::fromUtf8("_itc_max_Iter_Corner_LineEdit_"));

        horizontalLayout_13->addWidget(_itc_max_Iter_Corner_LineEdit_);


        verticalLayout_5->addLayout(horizontalLayout_13);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        _itc_epsilon_Corner_checkBox_2 = new QCheckBox(_itc_IterativeTreminationFlags_Corner_BOX);
        _itc_epsilon_Corner_checkBox_2->setObjectName(QString::fromUtf8("_itc_epsilon_Corner_checkBox_2"));
        _itc_epsilon_Corner_checkBox_2->setChecked(true);

        horizontalLayout_14->addWidget(_itc_epsilon_Corner_checkBox_2);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_14->addItem(horizontalSpacer_2);

        _itc_epsilon_Corner_LineEdit = new QLineEdit(_itc_IterativeTreminationFlags_Corner_BOX);
        _itc_epsilon_Corner_LineEdit->setObjectName(QString::fromUtf8("_itc_epsilon_Corner_LineEdit"));

        horizontalLayout_14->addWidget(_itc_epsilon_Corner_LineEdit);


        verticalLayout_5->addLayout(horizontalLayout_14);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_5->addItem(verticalSpacer_3);


        verticalLayout_4->addWidget(_itc_IterativeTreminationFlags_Corner_BOX);

        tabWidget->addTab(tabCornerFlags, QString());
        tabCalFlags = new QWidget();
        tabCalFlags->setObjectName(QString::fromUtf8("tabCalFlags"));
        verticalLayout_8 = new QVBoxLayout(tabCalFlags);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        _CalFlags_groupBox = new QGroupBox(tabCalFlags);
        _CalFlags_groupBox->setObjectName(QString::fromUtf8("_CalFlags_groupBox"));
        verticalLayout_7 = new QVBoxLayout(_CalFlags_groupBox);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        horizontalLayout_16 = new QHBoxLayout();
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        _useRationalModel_CB = new QCheckBox(_CalFlags_groupBox);
        _useRationalModel_CB->setObjectName(QString::fromUtf8("_useRationalModel_CB"));

        horizontalLayout_16->addWidget(_useRationalModel_CB);

        _useIntrinsicGuess_CB = new QCheckBox(_CalFlags_groupBox);
        _useIntrinsicGuess_CB->setObjectName(QString::fromUtf8("_useIntrinsicGuess_CB"));

        horizontalLayout_16->addWidget(_useIntrinsicGuess_CB);


        verticalLayout_7->addLayout(horizontalLayout_16);

        _Cam1Intrinsicload_container = new QHBoxLayout();
        _Cam1Intrinsicload_container->setObjectName(QString::fromUtf8("_Cam1Intrinsicload_container"));
        _intrinsicGuessCam1label = new QLabel(_CalFlags_groupBox);
        _intrinsicGuessCam1label->setObjectName(QString::fromUtf8("_intrinsicGuessCam1label"));

        _Cam1Intrinsicload_container->addWidget(_intrinsicGuessCam1label);

        _intrinsicGuessCam1Path = new QLineEdit(_CalFlags_groupBox);
        _intrinsicGuessCam1Path->setObjectName(QString::fromUtf8("_intrinsicGuessCam1Path"));

        _Cam1Intrinsicload_container->addWidget(_intrinsicGuessCam1Path);

        _intrinsicGuessCam1btnFileBrowse = new QToolButton(_CalFlags_groupBox);
        _intrinsicGuessCam1btnFileBrowse->setObjectName(QString::fromUtf8("_intrinsicGuessCam1btnFileBrowse"));

        _Cam1Intrinsicload_container->addWidget(_intrinsicGuessCam1btnFileBrowse);


        verticalLayout_7->addLayout(_Cam1Intrinsicload_container);

        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        _intrinsicGuessCam2label = new QLabel(_CalFlags_groupBox);
        _intrinsicGuessCam2label->setObjectName(QString::fromUtf8("_intrinsicGuessCam2label"));

        horizontalLayout_15->addWidget(_intrinsicGuessCam2label);

        _intrinsicGuessCam2Path = new QLineEdit(_CalFlags_groupBox);
        _intrinsicGuessCam2Path->setObjectName(QString::fromUtf8("_intrinsicGuessCam2Path"));

        horizontalLayout_15->addWidget(_intrinsicGuessCam2Path);

        _intrinsicGuessCam2btnFileBrowse = new QToolButton(_CalFlags_groupBox);
        _intrinsicGuessCam2btnFileBrowse->setObjectName(QString::fromUtf8("_intrinsicGuessCam2btnFileBrowse"));

        horizontalLayout_15->addWidget(_intrinsicGuessCam2btnFileBrowse);


        verticalLayout_7->addLayout(horizontalLayout_15);

        groupBox_inspectIntrinsic = new QGroupBox(_CalFlags_groupBox);
        groupBox_inspectIntrinsic->setObjectName(QString::fromUtf8("groupBox_inspectIntrinsic"));
        QSizePolicy sizePolicy7(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy7.setHorizontalStretch(0);
        sizePolicy7.setVerticalStretch(0);
        sizePolicy7.setHeightForWidth(groupBox_inspectIntrinsic->sizePolicy().hasHeightForWidth());
        groupBox_inspectIntrinsic->setSizePolicy(sizePolicy7);
        groupBox_inspectIntrinsic->setMinimumSize(QSize(0, 0));
        groupBox_inspectIntrinsic->setCheckable(true);
        verticalLayout_9 = new QVBoxLayout(groupBox_inspectIntrinsic);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));

        verticalLayout_7->addWidget(groupBox_inspectIntrinsic);

        _inspectCamTabs = new QTabWidget(_CalFlags_groupBox);
        _inspectCamTabs->setObjectName(QString::fromUtf8("_inspectCamTabs"));
        _inspectIntrinsicCamera1Tab = new QWidget();
        _inspectIntrinsicCamera1Tab->setObjectName(QString::fromUtf8("_inspectIntrinsicCamera1Tab"));
        verticalLayout_10 = new QVBoxLayout(_inspectIntrinsicCamera1Tab);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setObjectName(QString::fromUtf8("horizontalLayout_17"));
        _intrinsicReadFromFileBtnL = new QPushButton(_inspectIntrinsicCamera1Tab);
        _intrinsicReadFromFileBtnL->setObjectName(QString::fromUtf8("_intrinsicReadFromFileBtnL"));

        horizontalLayout_17->addWidget(_intrinsicReadFromFileBtnL);

        _intrinsicClearValsBtnL = new QPushButton(_inspectIntrinsicCamera1Tab);
        _intrinsicClearValsBtnL->setObjectName(QString::fromUtf8("_intrinsicClearValsBtnL"));

        horizontalLayout_17->addWidget(_intrinsicClearValsBtnL);


        verticalLayout_10->addLayout(horizontalLayout_17);

        _fixPrincipalPoint_CB_CL = new QCheckBox(_inspectIntrinsicCamera1Tab);
        _fixPrincipalPoint_CB_CL->setObjectName(QString::fromUtf8("_fixPrincipalPoint_CB_CL"));

        verticalLayout_10->addWidget(_fixPrincipalPoint_CB_CL);

        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setObjectName(QString::fromUtf8("horizontalLayout_19"));
        label_12 = new QLabel(_inspectIntrinsicCamera1Tab);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        horizontalLayout_19->addWidget(label_12);

        _fixPrincipalPoint_VALx_CL = new QLineEdit(_inspectIntrinsicCamera1Tab);
        _fixPrincipalPoint_VALx_CL->setObjectName(QString::fromUtf8("_fixPrincipalPoint_VALx_CL"));

        horizontalLayout_19->addWidget(_fixPrincipalPoint_VALx_CL);


        verticalLayout_10->addLayout(horizontalLayout_19);

        horizontalLayout_20 = new QHBoxLayout();
        horizontalLayout_20->setObjectName(QString::fromUtf8("horizontalLayout_20"));
        label_13 = new QLabel(_inspectIntrinsicCamera1Tab);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        horizontalLayout_20->addWidget(label_13);

        _fixPrincipalPoint_VALy_CL = new QLineEdit(_inspectIntrinsicCamera1Tab);
        _fixPrincipalPoint_VALy_CL->setObjectName(QString::fromUtf8("_fixPrincipalPoint_VALy_CL"));

        horizontalLayout_20->addWidget(_fixPrincipalPoint_VALy_CL);


        verticalLayout_10->addLayout(horizontalLayout_20);

        horizontalLayout_23 = new QHBoxLayout();
        horizontalLayout_23->setObjectName(QString::fromUtf8("horizontalLayout_23"));
        _fixFocalLength_CB_CL = new QCheckBox(_inspectIntrinsicCamera1Tab);
        _fixFocalLength_CB_CL->setObjectName(QString::fromUtf8("_fixFocalLength_CB_CL"));

        horizontalLayout_23->addWidget(_fixFocalLength_CB_CL);

        _fixAspectRatio_CB_CL = new QCheckBox(_inspectIntrinsicCamera1Tab);
        _fixAspectRatio_CB_CL->setObjectName(QString::fromUtf8("_fixAspectRatio_CB_CL"));

        horizontalLayout_23->addWidget(_fixAspectRatio_CB_CL);


        verticalLayout_10->addLayout(horizontalLayout_23);

        horizontalLayout_26 = new QHBoxLayout();
        horizontalLayout_26->setObjectName(QString::fromUtf8("horizontalLayout_26"));
        label_16 = new QLabel(_inspectIntrinsicCamera1Tab);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        horizontalLayout_26->addWidget(label_16);

        _fixFocalLength_VALx_CL = new QLineEdit(_inspectIntrinsicCamera1Tab);
        _fixFocalLength_VALx_CL->setObjectName(QString::fromUtf8("_fixFocalLength_VALx_CL"));

        horizontalLayout_26->addWidget(_fixFocalLength_VALx_CL);


        verticalLayout_10->addLayout(horizontalLayout_26);

        horizontalLayout_27 = new QHBoxLayout();
        horizontalLayout_27->setObjectName(QString::fromUtf8("horizontalLayout_27"));
        label_17 = new QLabel(_inspectIntrinsicCamera1Tab);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        horizontalLayout_27->addWidget(label_17);

        _fixFocalLength_VALy_CL = new QLineEdit(_inspectIntrinsicCamera1Tab);
        _fixFocalLength_VALy_CL->setObjectName(QString::fromUtf8("_fixFocalLength_VALy_CL"));

        horizontalLayout_27->addWidget(_fixFocalLength_VALy_CL);


        verticalLayout_10->addLayout(horizontalLayout_27);

        label_20 = new QLabel(_inspectIntrinsicCamera1Tab);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        verticalLayout_10->addWidget(label_20);

        _fixZeroTangentDistortion_CB_CL = new QCheckBox(_inspectIntrinsicCamera1Tab);
        _fixZeroTangentDistortion_CB_CL->setObjectName(QString::fromUtf8("_fixZeroTangentDistortion_CB_CL"));

        verticalLayout_10->addWidget(_fixZeroTangentDistortion_CB_CL);

        horizontalLayout_31 = new QHBoxLayout();
        horizontalLayout_31->setObjectName(QString::fromUtf8("horizontalLayout_31"));
        label_24 = new QLabel(_inspectIntrinsicCamera1Tab);
        label_24->setObjectName(QString::fromUtf8("label_24"));

        horizontalLayout_31->addWidget(label_24);

        _fix_P1_VAL_CL = new QLineEdit(_inspectIntrinsicCamera1Tab);
        _fix_P1_VAL_CL->setObjectName(QString::fromUtf8("_fix_P1_VAL_CL"));

        horizontalLayout_31->addWidget(_fix_P1_VAL_CL);


        verticalLayout_10->addLayout(horizontalLayout_31);

        horizontalLayout_32 = new QHBoxLayout();
        horizontalLayout_32->setObjectName(QString::fromUtf8("horizontalLayout_32"));
        label_25 = new QLabel(_inspectIntrinsicCamera1Tab);
        label_25->setObjectName(QString::fromUtf8("label_25"));

        horizontalLayout_32->addWidget(label_25);

        _fix_P2_VAL_CL = new QLineEdit(_inspectIntrinsicCamera1Tab);
        _fix_P2_VAL_CL->setObjectName(QString::fromUtf8("_fix_P2_VAL_CL"));

        horizontalLayout_32->addWidget(_fix_P2_VAL_CL);


        verticalLayout_10->addLayout(horizontalLayout_32);

        horizontalLayout_33 = new QHBoxLayout();
        horizontalLayout_33->setObjectName(QString::fromUtf8("horizontalLayout_33"));
        _fix_K1_CB_CL = new QCheckBox(_inspectIntrinsicCamera1Tab);
        _fix_K1_CB_CL->setObjectName(QString::fromUtf8("_fix_K1_CB_CL"));

        horizontalLayout_33->addWidget(_fix_K1_CB_CL);

        _fix_K1_VAL_CL = new QLineEdit(_inspectIntrinsicCamera1Tab);
        _fix_K1_VAL_CL->setObjectName(QString::fromUtf8("_fix_K1_VAL_CL"));

        horizontalLayout_33->addWidget(_fix_K1_VAL_CL);


        verticalLayout_10->addLayout(horizontalLayout_33);

        horizontalLayout_34 = new QHBoxLayout();
        horizontalLayout_34->setObjectName(QString::fromUtf8("horizontalLayout_34"));
        _fix_K2_CB_CL = new QCheckBox(_inspectIntrinsicCamera1Tab);
        _fix_K2_CB_CL->setObjectName(QString::fromUtf8("_fix_K2_CB_CL"));

        horizontalLayout_34->addWidget(_fix_K2_CB_CL);

        _fix_K2_VAL_CL = new QLineEdit(_inspectIntrinsicCamera1Tab);
        _fix_K2_VAL_CL->setObjectName(QString::fromUtf8("_fix_K2_VAL_CL"));

        horizontalLayout_34->addWidget(_fix_K2_VAL_CL);


        verticalLayout_10->addLayout(horizontalLayout_34);

        horizontalLayout_35 = new QHBoxLayout();
        horizontalLayout_35->setObjectName(QString::fromUtf8("horizontalLayout_35"));
        _fix_K3_CB_CL = new QCheckBox(_inspectIntrinsicCamera1Tab);
        _fix_K3_CB_CL->setObjectName(QString::fromUtf8("_fix_K3_CB_CL"));

        horizontalLayout_35->addWidget(_fix_K3_CB_CL);

        _fix_K3_VAL_CL = new QLineEdit(_inspectIntrinsicCamera1Tab);
        _fix_K3_VAL_CL->setObjectName(QString::fromUtf8("_fix_K3_VAL_CL"));

        horizontalLayout_35->addWidget(_fix_K3_VAL_CL);


        verticalLayout_10->addLayout(horizontalLayout_35);

        horizontalLayout_36 = new QHBoxLayout();
        horizontalLayout_36->setObjectName(QString::fromUtf8("horizontalLayout_36"));
        _fix_K4_CB_CL = new QCheckBox(_inspectIntrinsicCamera1Tab);
        _fix_K4_CB_CL->setObjectName(QString::fromUtf8("_fix_K4_CB_CL"));

        horizontalLayout_36->addWidget(_fix_K4_CB_CL);

        _fix_K4_VAL_CL = new QLineEdit(_inspectIntrinsicCamera1Tab);
        _fix_K4_VAL_CL->setObjectName(QString::fromUtf8("_fix_K4_VAL_CL"));

        horizontalLayout_36->addWidget(_fix_K4_VAL_CL);


        verticalLayout_10->addLayout(horizontalLayout_36);

        horizontalLayout_37 = new QHBoxLayout();
        horizontalLayout_37->setObjectName(QString::fromUtf8("horizontalLayout_37"));
        _fix_K5_CB_CL = new QCheckBox(_inspectIntrinsicCamera1Tab);
        _fix_K5_CB_CL->setObjectName(QString::fromUtf8("_fix_K5_CB_CL"));

        horizontalLayout_37->addWidget(_fix_K5_CB_CL);

        _fix_K5_VAL_CL = new QLineEdit(_inspectIntrinsicCamera1Tab);
        _fix_K5_VAL_CL->setObjectName(QString::fromUtf8("_fix_K5_VAL_CL"));

        horizontalLayout_37->addWidget(_fix_K5_VAL_CL);


        verticalLayout_10->addLayout(horizontalLayout_37);

        horizontalLayout_38 = new QHBoxLayout();
        horizontalLayout_38->setObjectName(QString::fromUtf8("horizontalLayout_38"));
        _fix_K6_CB_CL = new QCheckBox(_inspectIntrinsicCamera1Tab);
        _fix_K6_CB_CL->setObjectName(QString::fromUtf8("_fix_K6_CB_CL"));

        horizontalLayout_38->addWidget(_fix_K6_CB_CL);

        _fix_K6_VAL_CL = new QLineEdit(_inspectIntrinsicCamera1Tab);
        _fix_K6_VAL_CL->setObjectName(QString::fromUtf8("_fix_K6_VAL_CL"));

        horizontalLayout_38->addWidget(_fix_K6_VAL_CL);


        verticalLayout_10->addLayout(horizontalLayout_38);

        _inspectCamTabs->addTab(_inspectIntrinsicCamera1Tab, QString());
        _inspectIntrinsicCamera2Tab = new QWidget();
        _inspectIntrinsicCamera2Tab->setObjectName(QString::fromUtf8("_inspectIntrinsicCamera2Tab"));
        verticalLayout_11 = new QVBoxLayout(_inspectIntrinsicCamera2Tab);
        verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setObjectName(QString::fromUtf8("horizontalLayout_18"));
        _intrinsicReadFromFileBtnR = new QPushButton(_inspectIntrinsicCamera2Tab);
        _intrinsicReadFromFileBtnR->setObjectName(QString::fromUtf8("_intrinsicReadFromFileBtnR"));

        horizontalLayout_18->addWidget(_intrinsicReadFromFileBtnR);

        _intrinsicClearValsBtnR = new QPushButton(_inspectIntrinsicCamera2Tab);
        _intrinsicClearValsBtnR->setObjectName(QString::fromUtf8("_intrinsicClearValsBtnR"));

        horizontalLayout_18->addWidget(_intrinsicClearValsBtnR);


        verticalLayout_11->addLayout(horizontalLayout_18);

        _fixPrincipalPoint_CB_CR = new QCheckBox(_inspectIntrinsicCamera2Tab);
        _fixPrincipalPoint_CB_CR->setObjectName(QString::fromUtf8("_fixPrincipalPoint_CB_CR"));

        verticalLayout_11->addWidget(_fixPrincipalPoint_CB_CR);

        horizontalLayout_21 = new QHBoxLayout();
        horizontalLayout_21->setObjectName(QString::fromUtf8("horizontalLayout_21"));
        label_14 = new QLabel(_inspectIntrinsicCamera2Tab);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        horizontalLayout_21->addWidget(label_14);

        _fixPrincipalPoint_VALx_CR = new QLineEdit(_inspectIntrinsicCamera2Tab);
        _fixPrincipalPoint_VALx_CR->setObjectName(QString::fromUtf8("_fixPrincipalPoint_VALx_CR"));

        horizontalLayout_21->addWidget(_fixPrincipalPoint_VALx_CR);


        verticalLayout_11->addLayout(horizontalLayout_21);

        horizontalLayout_22 = new QHBoxLayout();
        horizontalLayout_22->setObjectName(QString::fromUtf8("horizontalLayout_22"));
        label_15 = new QLabel(_inspectIntrinsicCamera2Tab);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        horizontalLayout_22->addWidget(label_15);

        _fixPrincipalPoint_VALy_CR = new QLineEdit(_inspectIntrinsicCamera2Tab);
        _fixPrincipalPoint_VALy_CR->setObjectName(QString::fromUtf8("_fixPrincipalPoint_VALy_CR"));

        horizontalLayout_22->addWidget(_fixPrincipalPoint_VALy_CR);


        verticalLayout_11->addLayout(horizontalLayout_22);

        horizontalLayout_24 = new QHBoxLayout();
        horizontalLayout_24->setObjectName(QString::fromUtf8("horizontalLayout_24"));
        _fixFocalLength_CB_CR = new QCheckBox(_inspectIntrinsicCamera2Tab);
        _fixFocalLength_CB_CR->setObjectName(QString::fromUtf8("_fixFocalLength_CB_CR"));

        horizontalLayout_24->addWidget(_fixFocalLength_CB_CR);

        _fixAspectRatio_CB_CR = new QCheckBox(_inspectIntrinsicCamera2Tab);
        _fixAspectRatio_CB_CR->setObjectName(QString::fromUtf8("_fixAspectRatio_CB_CR"));

        horizontalLayout_24->addWidget(_fixAspectRatio_CB_CR);


        verticalLayout_11->addLayout(horizontalLayout_24);

        horizontalLayout_25 = new QHBoxLayout();
        horizontalLayout_25->setObjectName(QString::fromUtf8("horizontalLayout_25"));
        label_18 = new QLabel(_inspectIntrinsicCamera2Tab);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        horizontalLayout_25->addWidget(label_18);

        _fixFocalLength_VALx_CR = new QLineEdit(_inspectIntrinsicCamera2Tab);
        _fixFocalLength_VALx_CR->setObjectName(QString::fromUtf8("_fixFocalLength_VALx_CR"));

        horizontalLayout_25->addWidget(_fixFocalLength_VALx_CR);


        verticalLayout_11->addLayout(horizontalLayout_25);

        horizontalLayout_28 = new QHBoxLayout();
        horizontalLayout_28->setObjectName(QString::fromUtf8("horizontalLayout_28"));
        label_19 = new QLabel(_inspectIntrinsicCamera2Tab);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        horizontalLayout_28->addWidget(label_19);

        _fixFocalLength_VALy_CR = new QLineEdit(_inspectIntrinsicCamera2Tab);
        _fixFocalLength_VALy_CR->setObjectName(QString::fromUtf8("_fixFocalLength_VALy_CR"));

        horizontalLayout_28->addWidget(_fixFocalLength_VALy_CR);


        verticalLayout_11->addLayout(horizontalLayout_28);

        label_21 = new QLabel(_inspectIntrinsicCamera2Tab);
        label_21->setObjectName(QString::fromUtf8("label_21"));

        verticalLayout_11->addWidget(label_21);

        _fixZeroTangentDistortion_CB_CR = new QCheckBox(_inspectIntrinsicCamera2Tab);
        _fixZeroTangentDistortion_CB_CR->setObjectName(QString::fromUtf8("_fixZeroTangentDistortion_CB_CR"));

        verticalLayout_11->addWidget(_fixZeroTangentDistortion_CB_CR);

        horizontalLayout_29 = new QHBoxLayout();
        horizontalLayout_29->setObjectName(QString::fromUtf8("horizontalLayout_29"));
        label_22 = new QLabel(_inspectIntrinsicCamera2Tab);
        label_22->setObjectName(QString::fromUtf8("label_22"));

        horizontalLayout_29->addWidget(label_22);

        _fix_P1_VAL_CR = new QLineEdit(_inspectIntrinsicCamera2Tab);
        _fix_P1_VAL_CR->setObjectName(QString::fromUtf8("_fix_P1_VAL_CR"));

        horizontalLayout_29->addWidget(_fix_P1_VAL_CR);


        verticalLayout_11->addLayout(horizontalLayout_29);

        horizontalLayout_30 = new QHBoxLayout();
        horizontalLayout_30->setObjectName(QString::fromUtf8("horizontalLayout_30"));
        label_23 = new QLabel(_inspectIntrinsicCamera2Tab);
        label_23->setObjectName(QString::fromUtf8("label_23"));

        horizontalLayout_30->addWidget(label_23);

        _fix_P2_VAL_CR = new QLineEdit(_inspectIntrinsicCamera2Tab);
        _fix_P2_VAL_CR->setObjectName(QString::fromUtf8("_fix_P2_VAL_CR"));

        horizontalLayout_30->addWidget(_fix_P2_VAL_CR);


        verticalLayout_11->addLayout(horizontalLayout_30);

        horizontalLayout_39 = new QHBoxLayout();
        horizontalLayout_39->setObjectName(QString::fromUtf8("horizontalLayout_39"));
        _fix_K1_CB_CR = new QCheckBox(_inspectIntrinsicCamera2Tab);
        _fix_K1_CB_CR->setObjectName(QString::fromUtf8("_fix_K1_CB_CR"));

        horizontalLayout_39->addWidget(_fix_K1_CB_CR);

        _fix_K1_VAL_CR = new QLineEdit(_inspectIntrinsicCamera2Tab);
        _fix_K1_VAL_CR->setObjectName(QString::fromUtf8("_fix_K1_VAL_CR"));

        horizontalLayout_39->addWidget(_fix_K1_VAL_CR);


        verticalLayout_11->addLayout(horizontalLayout_39);

        horizontalLayout_40 = new QHBoxLayout();
        horizontalLayout_40->setObjectName(QString::fromUtf8("horizontalLayout_40"));
        _fix_K2_CB_CR = new QCheckBox(_inspectIntrinsicCamera2Tab);
        _fix_K2_CB_CR->setObjectName(QString::fromUtf8("_fix_K2_CB_CR"));

        horizontalLayout_40->addWidget(_fix_K2_CB_CR);

        _fix_K2_VAL_CR = new QLineEdit(_inspectIntrinsicCamera2Tab);
        _fix_K2_VAL_CR->setObjectName(QString::fromUtf8("_fix_K2_VAL_CR"));

        horizontalLayout_40->addWidget(_fix_K2_VAL_CR);


        verticalLayout_11->addLayout(horizontalLayout_40);

        horizontalLayout_41 = new QHBoxLayout();
        horizontalLayout_41->setObjectName(QString::fromUtf8("horizontalLayout_41"));
        _fix_K3_CB_CR = new QCheckBox(_inspectIntrinsicCamera2Tab);
        _fix_K3_CB_CR->setObjectName(QString::fromUtf8("_fix_K3_CB_CR"));

        horizontalLayout_41->addWidget(_fix_K3_CB_CR);

        _fix_K3_VAL_CR = new QLineEdit(_inspectIntrinsicCamera2Tab);
        _fix_K3_VAL_CR->setObjectName(QString::fromUtf8("_fix_K3_VAL_CR"));

        horizontalLayout_41->addWidget(_fix_K3_VAL_CR);


        verticalLayout_11->addLayout(horizontalLayout_41);

        horizontalLayout_42 = new QHBoxLayout();
        horizontalLayout_42->setObjectName(QString::fromUtf8("horizontalLayout_42"));
        _fix_K4_CB_CR = new QCheckBox(_inspectIntrinsicCamera2Tab);
        _fix_K4_CB_CR->setObjectName(QString::fromUtf8("_fix_K4_CB_CR"));

        horizontalLayout_42->addWidget(_fix_K4_CB_CR);

        _fix_K4_VAL_CR = new QLineEdit(_inspectIntrinsicCamera2Tab);
        _fix_K4_VAL_CR->setObjectName(QString::fromUtf8("_fix_K4_VAL_CR"));

        horizontalLayout_42->addWidget(_fix_K4_VAL_CR);


        verticalLayout_11->addLayout(horizontalLayout_42);

        horizontalLayout_43 = new QHBoxLayout();
        horizontalLayout_43->setObjectName(QString::fromUtf8("horizontalLayout_43"));
        _fix_K5_CB_CR = new QCheckBox(_inspectIntrinsicCamera2Tab);
        _fix_K5_CB_CR->setObjectName(QString::fromUtf8("_fix_K5_CB_CR"));

        horizontalLayout_43->addWidget(_fix_K5_CB_CR);

        _fix_K5_VAL_CR = new QLineEdit(_inspectIntrinsicCamera2Tab);
        _fix_K5_VAL_CR->setObjectName(QString::fromUtf8("_fix_K5_VAL_CR"));

        horizontalLayout_43->addWidget(_fix_K5_VAL_CR);


        verticalLayout_11->addLayout(horizontalLayout_43);

        horizontalLayout_44 = new QHBoxLayout();
        horizontalLayout_44->setObjectName(QString::fromUtf8("horizontalLayout_44"));
        _fix_K6_CB_CR = new QCheckBox(_inspectIntrinsicCamera2Tab);
        _fix_K6_CB_CR->setObjectName(QString::fromUtf8("_fix_K6_CB_CR"));

        horizontalLayout_44->addWidget(_fix_K6_CB_CR);

        _fix_K6_VAL_CR = new QLineEdit(_inspectIntrinsicCamera2Tab);
        _fix_K6_VAL_CR->setObjectName(QString::fromUtf8("_fix_K6_VAL_CR"));

        horizontalLayout_44->addWidget(_fix_K6_VAL_CR);


        verticalLayout_11->addLayout(horizontalLayout_44);

        _inspectCamTabs->addTab(_inspectIntrinsicCamera2Tab, QString());

        verticalLayout_7->addWidget(_inspectCamTabs);


        verticalLayout_8->addWidget(_CalFlags_groupBox);

        groupBox_StereoCalFlags = new QGroupBox(tabCalFlags);
        groupBox_StereoCalFlags->setObjectName(QString::fromUtf8("groupBox_StereoCalFlags"));
        groupBox_StereoCalFlags->setCheckable(true);
        groupBox_StereoCalFlags->setChecked(false);
        verticalLayout_6 = new QVBoxLayout(groupBox_StereoCalFlags);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        _fixIntrinsicStereo_CB = new QCheckBox(groupBox_StereoCalFlags);
        _fixIntrinsicStereo_CB->setObjectName(QString::fromUtf8("_fixIntrinsicStereo_CB"));

        verticalLayout_6->addWidget(_fixIntrinsicStereo_CB);

        _sameFocalLengthStereo_CB = new QCheckBox(groupBox_StereoCalFlags);
        _sameFocalLengthStereo_CB->setObjectName(QString::fromUtf8("_sameFocalLengthStereo_CB"));

        verticalLayout_6->addWidget(_sameFocalLengthStereo_CB);

        _itc_IterativeTreminationFlags_Stereo_BOX = new QGroupBox(groupBox_StereoCalFlags);
        _itc_IterativeTreminationFlags_Stereo_BOX->setObjectName(QString::fromUtf8("_itc_IterativeTreminationFlags_Stereo_BOX"));
        _itc_IterativeTreminationFlags_Stereo_BOX->setCheckable(true);
        _itc_IterativeTreminationFlags_Stereo_BOX->setChecked(false);
        verticalLayout_12 = new QVBoxLayout(_itc_IterativeTreminationFlags_Stereo_BOX);
        verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
        label_26 = new QLabel(_itc_IterativeTreminationFlags_Stereo_BOX);
        label_26->setObjectName(QString::fromUtf8("label_26"));

        verticalLayout_12->addWidget(label_26);

        horizontalLayout_45 = new QHBoxLayout();
        horizontalLayout_45->setObjectName(QString::fromUtf8("horizontalLayout_45"));
        _itc_iterations_Stereo_checkBox = new QCheckBox(_itc_IterativeTreminationFlags_Stereo_BOX);
        _itc_iterations_Stereo_checkBox->setObjectName(QString::fromUtf8("_itc_iterations_Stereo_checkBox"));

        horizontalLayout_45->addWidget(_itc_iterations_Stereo_checkBox);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_45->addItem(horizontalSpacer_3);

        _itc_max_Iter_Stereo_LineEdit = new QLineEdit(_itc_IterativeTreminationFlags_Stereo_BOX);
        _itc_max_Iter_Stereo_LineEdit->setObjectName(QString::fromUtf8("_itc_max_Iter_Stereo_LineEdit"));

        horizontalLayout_45->addWidget(_itc_max_Iter_Stereo_LineEdit);


        verticalLayout_12->addLayout(horizontalLayout_45);

        horizontalLayout_46 = new QHBoxLayout();
        horizontalLayout_46->setObjectName(QString::fromUtf8("horizontalLayout_46"));
        _itc_epsilon_Stereo_checkBox = new QCheckBox(_itc_IterativeTreminationFlags_Stereo_BOX);
        _itc_epsilon_Stereo_checkBox->setObjectName(QString::fromUtf8("_itc_epsilon_Stereo_checkBox"));

        horizontalLayout_46->addWidget(_itc_epsilon_Stereo_checkBox);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_46->addItem(horizontalSpacer_4);

        _itc_epsilon_Stereo_LineEdit = new QLineEdit(_itc_IterativeTreminationFlags_Stereo_BOX);
        _itc_epsilon_Stereo_LineEdit->setObjectName(QString::fromUtf8("_itc_epsilon_Stereo_LineEdit"));

        horizontalLayout_46->addWidget(_itc_epsilon_Stereo_LineEdit);


        verticalLayout_12->addLayout(horizontalLayout_46);


        verticalLayout_6->addWidget(_itc_IterativeTreminationFlags_Stereo_BOX);


        verticalLayout_8->addWidget(groupBox_StereoCalFlags);

        verticalSpacer = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_8->addItem(verticalSpacer);

        tabWidget->addTab(tabCalFlags, QString());
        tabRectFlags = new QWidget();
        tabRectFlags->setObjectName(QString::fromUtf8("tabRectFlags"));
        verticalLayout_13 = new QVBoxLayout(tabRectFlags);
        verticalLayout_13->setObjectName(QString::fromUtf8("verticalLayout_13"));
        groupBox_2 = new QGroupBox(tabRectFlags);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        verticalLayout_14 = new QVBoxLayout(groupBox_2);
        verticalLayout_14->setObjectName(QString::fromUtf8("verticalLayout_14"));
        horizontalLayout_47 = new QHBoxLayout();
        horizontalLayout_47->setObjectName(QString::fromUtf8("horizontalLayout_47"));
        _rectAlgorithm_Label = new QLabel(groupBox_2);
        _rectAlgorithm_Label->setObjectName(QString::fromUtf8("_rectAlgorithm_Label"));

        horizontalLayout_47->addWidget(_rectAlgorithm_Label);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_47->addItem(horizontalSpacer_5);

        _rectAlgorithm_Combo = new QComboBox(groupBox_2);
        _rectAlgorithm_Combo->setObjectName(QString::fromUtf8("_rectAlgorithm_Combo"));

        horizontalLayout_47->addWidget(_rectAlgorithm_Combo);


        verticalLayout_14->addLayout(horizontalLayout_47);

        horizontalLayout_48 = new QHBoxLayout();
        horizontalLayout_48->setObjectName(QString::fromUtf8("horizontalLayout_48"));
        _rectFundamentalLabel = new QLabel(groupBox_2);
        _rectFundamentalLabel->setObjectName(QString::fromUtf8("_rectFundamentalLabel"));

        horizontalLayout_48->addWidget(_rectFundamentalLabel);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_48->addItem(horizontalSpacer_6);

        _rectFundamental_Combo = new QComboBox(groupBox_2);
        _rectFundamental_Combo->setObjectName(QString::fromUtf8("_rectFundamental_Combo"));

        horizontalLayout_48->addWidget(_rectFundamental_Combo);


        verticalLayout_14->addLayout(horizontalLayout_48);

        horizontalLayout_49 = new QHBoxLayout();
        horizontalLayout_49->setObjectName(QString::fromUtf8("horizontalLayout_49"));
        _rectParam1_label = new QLabel(groupBox_2);
        _rectParam1_label->setObjectName(QString::fromUtf8("_rectParam1_label"));

        horizontalLayout_49->addWidget(_rectParam1_label);

        _rectParam1_VAL = new QLineEdit(groupBox_2);
        _rectParam1_VAL->setObjectName(QString::fromUtf8("_rectParam1_VAL"));

        horizontalLayout_49->addWidget(_rectParam1_VAL);

        _rectParam2_label = new QLabel(groupBox_2);
        _rectParam2_label->setObjectName(QString::fromUtf8("_rectParam2_label"));

        horizontalLayout_49->addWidget(_rectParam2_label);

        _rectParam2_VAL = new QLineEdit(groupBox_2);
        _rectParam2_VAL->setObjectName(QString::fromUtf8("_rectParam2_VAL"));

        horizontalLayout_49->addWidget(_rectParam2_VAL);


        verticalLayout_14->addLayout(horizontalLayout_49);

        horizontalLayout_50 = new QHBoxLayout();
        horizontalLayout_50->setObjectName(QString::fromUtf8("horizontalLayout_50"));
        _rectAlpha_CB = new QCheckBox(groupBox_2);
        _rectAlpha_CB->setObjectName(QString::fromUtf8("_rectAlpha_CB"));

        horizontalLayout_50->addWidget(_rectAlpha_CB);

        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_50->addItem(horizontalSpacer_7);

        _rectAlpha_VAL = new QDoubleSpinBox(groupBox_2);
        _rectAlpha_VAL->setObjectName(QString::fromUtf8("_rectAlpha_VAL"));
        _rectAlpha_VAL->setDecimals(5);

        horizontalLayout_50->addWidget(_rectAlpha_VAL);


        verticalLayout_14->addLayout(horizontalLayout_50);

        _zeroRectifyDisparity_CB = new QCheckBox(groupBox_2);
        _zeroRectifyDisparity_CB->setObjectName(QString::fromUtf8("_zeroRectifyDisparity_CB"));

        verticalLayout_14->addWidget(_zeroRectifyDisparity_CB);

        _rectDisplayValidRect_CB = new QCheckBox(groupBox_2);
        _rectDisplayValidRect_CB->setObjectName(QString::fromUtf8("_rectDisplayValidRect_CB"));

        verticalLayout_14->addWidget(_rectDisplayValidRect_CB);

        _displayRectification_CB = new QCheckBox(groupBox_2);
        _displayRectification_CB->setObjectName(QString::fromUtf8("_displayRectification_CB"));

        verticalLayout_14->addWidget(_displayRectification_CB);

        _saveRectification_CB = new QCheckBox(groupBox_2);
        _saveRectification_CB->setObjectName(QString::fromUtf8("_saveRectification_CB"));

        verticalLayout_14->addWidget(_saveRectification_CB);

        _rectAllImgsInFolder = new QCheckBox(groupBox_2);
        _rectAllImgsInFolder->setObjectName(QString::fromUtf8("_rectAllImgsInFolder"));

        verticalLayout_14->addWidget(_rectAllImgsInFolder);

        horizontalLayout_51 = new QHBoxLayout();
        horizontalLayout_51->setObjectName(QString::fromUtf8("horizontalLayout_51"));
        _rectSavePathLeftLabel = new QLabel(groupBox_2);
        _rectSavePathLeftLabel->setObjectName(QString::fromUtf8("_rectSavePathLeftLabel"));

        horizontalLayout_51->addWidget(_rectSavePathLeftLabel);

        horizontalSpacer_8 = new QSpacerItem(25, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout_51->addItem(horizontalSpacer_8);

        _rectificationSavePathTxtL = new QLineEdit(groupBox_2);
        _rectificationSavePathTxtL->setObjectName(QString::fromUtf8("_rectificationSavePathTxtL"));

        horizontalLayout_51->addWidget(_rectificationSavePathTxtL);

        _rectificationSavePathBtnL = new QToolButton(groupBox_2);
        _rectificationSavePathBtnL->setObjectName(QString::fromUtf8("_rectificationSavePathBtnL"));

        horizontalLayout_51->addWidget(_rectificationSavePathBtnL);


        verticalLayout_14->addLayout(horizontalLayout_51);

        horizontalLayout_52 = new QHBoxLayout();
        horizontalLayout_52->setObjectName(QString::fromUtf8("horizontalLayout_52"));
        _rectSavePathRightLabel = new QLabel(groupBox_2);
        _rectSavePathRightLabel->setObjectName(QString::fromUtf8("_rectSavePathRightLabel"));

        horizontalLayout_52->addWidget(_rectSavePathRightLabel);

        horizontalSpacer_9 = new QSpacerItem(17, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout_52->addItem(horizontalSpacer_9);

        _rectificationSavePathTxtR = new QLineEdit(groupBox_2);
        _rectificationSavePathTxtR->setObjectName(QString::fromUtf8("_rectificationSavePathTxtR"));

        horizontalLayout_52->addWidget(_rectificationSavePathTxtR);

        _rectificationSavePathBtnR = new QToolButton(groupBox_2);
        _rectificationSavePathBtnR->setObjectName(QString::fromUtf8("_rectificationSavePathBtnR"));

        horizontalLayout_52->addWidget(_rectificationSavePathBtnR);


        verticalLayout_14->addLayout(horizontalLayout_52);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_14->addItem(verticalSpacer_2);


        verticalLayout_13->addWidget(groupBox_2);

        tabWidget->addTab(tabRectFlags, QString());
        tabNarrowStereo = new QWidget();
        tabNarrowStereo->setObjectName(QString::fromUtf8("tabNarrowStereo"));
        layoutWidget = new QWidget(tabNarrowStereo);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(9, 9, 431, 227));
        gridLayout_2 = new QGridLayout(layoutWidget);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        label_10 = new QLabel(layoutWidget);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_2->addWidget(label_10, 0, 0, 1, 1);

        lineEditNarrowIntrinsicLeft = new QLineEdit(layoutWidget);
        lineEditNarrowIntrinsicLeft->setObjectName(QString::fromUtf8("lineEditNarrowIntrinsicLeft"));

        gridLayout_2->addWidget(lineEditNarrowIntrinsicLeft, 0, 1, 1, 1);

        btnFileBrowserNSIntrinsicLeft = new QToolButton(layoutWidget);
        btnFileBrowserNSIntrinsicLeft->setObjectName(QString::fromUtf8("btnFileBrowserNSIntrinsicLeft"));

        gridLayout_2->addWidget(btnFileBrowserNSIntrinsicLeft, 0, 2, 1, 1);

        label_11 = new QLabel(layoutWidget);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_2->addWidget(label_11, 1, 0, 1, 1);

        lineEditNarrowIntrinsicRight = new QLineEdit(layoutWidget);
        lineEditNarrowIntrinsicRight->setObjectName(QString::fromUtf8("lineEditNarrowIntrinsicRight"));

        gridLayout_2->addWidget(lineEditNarrowIntrinsicRight, 1, 1, 1, 1);

        btnFileBrowserNSIntrinsicRight = new QToolButton(layoutWidget);
        btnFileBrowserNSIntrinsicRight->setObjectName(QString::fromUtf8("btnFileBrowserNSIntrinsicRight"));

        gridLayout_2->addWidget(btnFileBrowserNSIntrinsicRight, 1, 2, 1, 1);

        label_27 = new QLabel(layoutWidget);
        label_27->setObjectName(QString::fromUtf8("label_27"));

        gridLayout_2->addWidget(label_27, 2, 0, 1, 1);

        lineEditNarrowImageLeft = new QLineEdit(layoutWidget);
        lineEditNarrowImageLeft->setObjectName(QString::fromUtf8("lineEditNarrowImageLeft"));

        gridLayout_2->addWidget(lineEditNarrowImageLeft, 2, 1, 1, 1);

        btnFileBrowserNSLeftImage = new QToolButton(layoutWidget);
        btnFileBrowserNSLeftImage->setObjectName(QString::fromUtf8("btnFileBrowserNSLeftImage"));

        gridLayout_2->addWidget(btnFileBrowserNSLeftImage, 2, 2, 1, 1);

        label_28 = new QLabel(layoutWidget);
        label_28->setObjectName(QString::fromUtf8("label_28"));

        gridLayout_2->addWidget(label_28, 3, 0, 1, 1);

        lineEditNarrowImageRight = new QLineEdit(layoutWidget);
        lineEditNarrowImageRight->setObjectName(QString::fromUtf8("lineEditNarrowImageRight"));

        gridLayout_2->addWidget(lineEditNarrowImageRight, 3, 1, 1, 1);

        btnFileBrowserNSRightImage = new QToolButton(layoutWidget);
        btnFileBrowserNSRightImage->setObjectName(QString::fromUtf8("btnFileBrowserNSRightImage"));

        gridLayout_2->addWidget(btnFileBrowserNSRightImage, 3, 2, 1, 1);

        label_30 = new QLabel(layoutWidget);
        label_30->setObjectName(QString::fromUtf8("label_30"));

        gridLayout_2->addWidget(label_30, 4, 0, 1, 1);

        lineEditNarrowOutputLeft = new QLineEdit(layoutWidget);
        lineEditNarrowOutputLeft->setObjectName(QString::fromUtf8("lineEditNarrowOutputLeft"));

        gridLayout_2->addWidget(lineEditNarrowOutputLeft, 4, 1, 1, 1);

        label_31 = new QLabel(layoutWidget);
        label_31->setObjectName(QString::fromUtf8("label_31"));

        gridLayout_2->addWidget(label_31, 5, 0, 1, 1);

        lineEditNarrowOutputRight = new QLineEdit(layoutWidget);
        lineEditNarrowOutputRight->setObjectName(QString::fromUtf8("lineEditNarrowOutputRight"));

        gridLayout_2->addWidget(lineEditNarrowOutputRight, 5, 1, 1, 1);

        pushButtonNarrowCompute = new QPushButton(layoutWidget);
        pushButtonNarrowCompute->setObjectName(QString::fromUtf8("pushButtonNarrowCompute"));

        gridLayout_2->addWidget(pushButtonNarrowCompute, 6, 1, 1, 1);

        tabWidget->addTab(tabNarrowStereo, QString());

        verticalLayout_3->addWidget(tabWidget);

        scrollArea->setWidget(scrollAreaWidgetContents);

        gridLayout_3->addWidget(scrollArea, 0, 0, 1, 1);

        verticalLayout_28 = new QVBoxLayout();
        verticalLayout_28->setObjectName(QString::fromUtf8("verticalLayout_28"));
        horizontalLayout_103 = new QHBoxLayout();
        horizontalLayout_103->setObjectName(QString::fromUtf8("horizontalLayout_103"));

        verticalLayout_28->addLayout(horizontalLayout_103);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_28->addItem(verticalSpacer_4);

        verticalLayout_16 = new QVBoxLayout();
        verticalLayout_16->setObjectName(QString::fromUtf8("verticalLayout_16"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(8);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setSizeConstraint(QLayout::SetDefaultConstraint);
        horizontalLayout_2->setContentsMargins(0, -1, -1, -1);
        _doSaveResultsBtn = new QPushButton(tabCameraCalib);
        _doSaveResultsBtn->setObjectName(QString::fromUtf8("_doSaveResultsBtn"));
        QSizePolicy sizePolicy8(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy8.setHorizontalStretch(0);
        sizePolicy8.setVerticalStretch(0);
        sizePolicy8.setHeightForWidth(_doSaveResultsBtn->sizePolicy().hasHeightForWidth());
        _doSaveResultsBtn->setSizePolicy(sizePolicy8);

        horizontalLayout_2->addWidget(_doSaveResultsBtn);

        _clearAllData = new QPushButton(tabCameraCalib);
        _clearAllData->setObjectName(QString::fromUtf8("_clearAllData"));
        sizePolicy8.setHeightForWidth(_clearAllData->sizePolicy().hasHeightForWidth());
        _clearAllData->setSizePolicy(sizePolicy8);

        horizontalLayout_2->addWidget(_clearAllData);

        _commitROS = new QPushButton(tabCameraCalib);
        _commitROS->setObjectName(QString::fromUtf8("_commitROS"));
        sizePolicy8.setHeightForWidth(_commitROS->sizePolicy().hasHeightForWidth());
        _commitROS->setSizePolicy(sizePolicy8);

        horizontalLayout_2->addWidget(_commitROS);

        _cancelBtn = new QPushButton(tabCameraCalib);
        _cancelBtn->setObjectName(QString::fromUtf8("_cancelBtn"));
        sizePolicy8.setHeightForWidth(_cancelBtn->sizePolicy().hasHeightForWidth());
        _cancelBtn->setSizePolicy(sizePolicy8);

        horizontalLayout_2->addWidget(_cancelBtn);


        verticalLayout_16->addLayout(horizontalLayout_2);

        horizontalLayout_105 = new QHBoxLayout();
        horizontalLayout_105->setSpacing(8);
        horizontalLayout_105->setObjectName(QString::fromUtf8("horizontalLayout_105"));
        horizontalLayout_105->setSizeConstraint(QLayout::SetDefaultConstraint);
        horizontalLayout_105->setContentsMargins(-1, -1, -1, 0);
        _doDetectCornersBtn = new QPushButton(tabCameraCalib);
        _doDetectCornersBtn->setObjectName(QString::fromUtf8("_doDetectCornersBtn"));
        sizePolicy8.setHeightForWidth(_doDetectCornersBtn->sizePolicy().hasHeightForWidth());
        _doDetectCornersBtn->setSizePolicy(sizePolicy8);

        horizontalLayout_105->addWidget(_doDetectCornersBtn);

        _doCalibrateBtn = new QPushButton(tabCameraCalib);
        _doCalibrateBtn->setObjectName(QString::fromUtf8("_doCalibrateBtn"));
        sizePolicy8.setHeightForWidth(_doCalibrateBtn->sizePolicy().hasHeightForWidth());
        _doCalibrateBtn->setSizePolicy(sizePolicy8);

        horizontalLayout_105->addWidget(_doCalibrateBtn);

        _doRectifyBtn = new QPushButton(tabCameraCalib);
        _doRectifyBtn->setObjectName(QString::fromUtf8("_doRectifyBtn"));
        sizePolicy8.setHeightForWidth(_doRectifyBtn->sizePolicy().hasHeightForWidth());
        _doRectifyBtn->setSizePolicy(sizePolicy8);

        horizontalLayout_105->addWidget(_doRectifyBtn);

        _doAllBtn = new QPushButton(tabCameraCalib);
        _doAllBtn->setObjectName(QString::fromUtf8("_doAllBtn"));
        _doAllBtn->setEnabled(true);
        sizePolicy8.setHeightForWidth(_doAllBtn->sizePolicy().hasHeightForWidth());
        _doAllBtn->setSizePolicy(sizePolicy8);

        horizontalLayout_105->addWidget(_doAllBtn);


        verticalLayout_16->addLayout(horizontalLayout_105);


        verticalLayout_28->addLayout(verticalLayout_16);


        gridLayout_3->addLayout(verticalLayout_28, 1, 0, 1, 1);

        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/images/checker64.jpg"), QSize(), QIcon::Normal, QIcon::Off);
        tabWidget_2->addTab(tabCameraCalib, icon1, QString());
        tabHandEye = new QWidget();
        tabHandEye->setObjectName(QString::fromUtf8("tabHandEye"));
        gridLayout_5 = new QGridLayout(tabHandEye);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        scrollArea_2 = new QScrollArea(tabHandEye);
        scrollArea_2->setObjectName(QString::fromUtf8("scrollArea_2"));
        sizePolicy3.setHeightForWidth(scrollArea_2->sizePolicy().hasHeightForWidth());
        scrollArea_2->setSizePolicy(sizePolicy3);
        scrollArea_2->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        scrollArea_2->setWidgetResizable(true);
        scrollAreaWidgetContents_2 = new QWidget();
        scrollAreaWidgetContents_2->setObjectName(QString::fromUtf8("scrollAreaWidgetContents_2"));
        scrollAreaWidgetContents_2->setGeometry(QRect(0, -149, 464, 777));
        verticalLayout_15 = new QVBoxLayout(scrollAreaWidgetContents_2);
        verticalLayout_15->setObjectName(QString::fromUtf8("verticalLayout_15"));
        label_29 = new QLabel(scrollAreaWidgetContents_2);
        label_29->setObjectName(QString::fromUtf8("label_29"));

        verticalLayout_15->addWidget(label_29);

        comboSelectCalibType_2 = new QComboBox(scrollAreaWidgetContents_2);
        comboSelectCalibType_2->setObjectName(QString::fromUtf8("comboSelectCalibType_2"));

        verticalLayout_15->addWidget(comboSelectCalibType_2);

        labelPreviewSize_2 = new QLabel(scrollAreaWidgetContents_2);
        labelPreviewSize_2->setObjectName(QString::fromUtf8("labelPreviewSize_2"));

        verticalLayout_15->addWidget(labelPreviewSize_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        comboPreviewSize_2 = new QComboBox(scrollAreaWidgetContents_2);
        comboPreviewSize_2->setObjectName(QString::fromUtf8("comboPreviewSize_2"));
        sizePolicy4.setHeightForWidth(comboPreviewSize_2->sizePolicy().hasHeightForWidth());
        comboPreviewSize_2->setSizePolicy(sizePolicy4);
        comboPreviewSize_2->setMinimumSize(QSize(100, 20));
        comboPreviewSize_2->setMaximumSize(QSize(2000, 20));

        horizontalLayout->addWidget(comboPreviewSize_2);

        lineEditPreviewWidth_2 = new QLineEdit(scrollAreaWidgetContents_2);
        lineEditPreviewWidth_2->setObjectName(QString::fromUtf8("lineEditPreviewWidth_2"));

        horizontalLayout->addWidget(lineEditPreviewWidth_2);

        lineEditPreviewHeight_2 = new QLineEdit(scrollAreaWidgetContents_2);
        lineEditPreviewHeight_2->setObjectName(QString::fromUtf8("lineEditPreviewHeight_2"));

        horizontalLayout->addWidget(lineEditPreviewHeight_2);


        verticalLayout_15->addLayout(horizontalLayout);

        label_32 = new QLabel(scrollAreaWidgetContents_2);
        label_32->setObjectName(QString::fromUtf8("label_32"));

        verticalLayout_15->addWidget(label_32);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        _fileFilterL_label_2 = new QLabel(scrollAreaWidgetContents_2);
        _fileFilterL_label_2->setObjectName(QString::fromUtf8("_fileFilterL_label_2"));
        _fileFilterL_label_2->setMinimumSize(QSize(37, 0));

        horizontalLayout_3->addWidget(_fileFilterL_label_2);

        lineEditNameFilterL_2 = new QLineEdit(scrollAreaWidgetContents_2);
        lineEditNameFilterL_2->setObjectName(QString::fromUtf8("lineEditNameFilterL_2"));
        sizePolicy5.setHeightForWidth(lineEditNameFilterL_2->sizePolicy().hasHeightForWidth());
        lineEditNameFilterL_2->setSizePolicy(sizePolicy5);

        horizontalLayout_3->addWidget(lineEditNameFilterL_2);

        imgPathL_2 = new QLineEdit(scrollAreaWidgetContents_2);
        imgPathL_2->setObjectName(QString::fromUtf8("imgPathL_2"));

        horizontalLayout_3->addWidget(imgPathL_2);

        btnFileBrowseL_2 = new QToolButton(scrollAreaWidgetContents_2);
        btnFileBrowseL_2->setObjectName(QString::fromUtf8("btnFileBrowseL_2"));

        horizontalLayout_3->addWidget(btnFileBrowseL_2);


        verticalLayout_15->addLayout(horizontalLayout_3);

        groupBox_4 = new QGroupBox(scrollAreaWidgetContents_2);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        formLayout_2 = new QFormLayout(groupBox_4);
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        formLayout_2->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        label_33 = new QLabel(groupBox_4);
        label_33->setObjectName(QString::fromUtf8("label_33"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, label_33);

        comboBoardType_2 = new QComboBox(groupBox_4);
        comboBoardType_2->setObjectName(QString::fromUtf8("comboBoardType_2"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, comboBoardType_2);

        label_34 = new QLabel(groupBox_4);
        label_34->setObjectName(QString::fromUtf8("label_34"));

        formLayout_2->setWidget(1, QFormLayout::LabelRole, label_34);

        horizontalLayout_53 = new QHBoxLayout();
        horizontalLayout_53->setObjectName(QString::fromUtf8("horizontalLayout_53"));
        spinBoxBoardSizeX_2 = new QSpinBox(groupBox_4);
        spinBoxBoardSizeX_2->setObjectName(QString::fromUtf8("spinBoxBoardSizeX_2"));
        spinBoxBoardSizeX_2->setValue(10);

        horizontalLayout_53->addWidget(spinBoxBoardSizeX_2);

        spinBoxBoardSizeY_2 = new QSpinBox(groupBox_4);
        spinBoxBoardSizeY_2->setObjectName(QString::fromUtf8("spinBoxBoardSizeY_2"));
        spinBoxBoardSizeY_2->setValue(6);

        horizontalLayout_53->addWidget(spinBoxBoardSizeY_2);


        formLayout_2->setLayout(1, QFormLayout::FieldRole, horizontalLayout_53);

        label_35 = new QLabel(groupBox_4);
        label_35->setObjectName(QString::fromUtf8("label_35"));

        formLayout_2->setWidget(2, QFormLayout::LabelRole, label_35);

        horizontalLayout_54 = new QHBoxLayout();
        horizontalLayout_54->setObjectName(QString::fromUtf8("horizontalLayout_54"));
        _squarePhysicalSize_SPINB_2 = new QDoubleSpinBox(groupBox_4);
        _squarePhysicalSize_SPINB_2->setObjectName(QString::fromUtf8("_squarePhysicalSize_SPINB_2"));
        _squarePhysicalSize_SPINB_2->setDecimals(5);
        _squarePhysicalSize_SPINB_2->setMaximum(1e+09);
        _squarePhysicalSize_SPINB_2->setValue(100);

        horizontalLayout_54->addWidget(_squarePhysicalSize_SPINB_2);


        formLayout_2->setLayout(2, QFormLayout::FieldRole, horizontalLayout_54);


        verticalLayout_15->addWidget(groupBox_4);

        labelPreviewSize_3 = new QLabel(scrollAreaWidgetContents_2);
        labelPreviewSize_3->setObjectName(QString::fromUtf8("labelPreviewSize_3"));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        labelPreviewSize_3->setFont(font);

        verticalLayout_15->addWidget(labelPreviewSize_3);

        _Cam1Intrinsicload_container_2 = new QHBoxLayout();
        _Cam1Intrinsicload_container_2->setObjectName(QString::fromUtf8("_Cam1Intrinsicload_container_2"));
        _intrinsicGuessCam1label_2 = new QLabel(scrollAreaWidgetContents_2);
        _intrinsicGuessCam1label_2->setObjectName(QString::fromUtf8("_intrinsicGuessCam1label_2"));

        _Cam1Intrinsicload_container_2->addWidget(_intrinsicGuessCam1label_2);

        _intrinsicGuessCam1Path_2 = new QLineEdit(scrollAreaWidgetContents_2);
        _intrinsicGuessCam1Path_2->setObjectName(QString::fromUtf8("_intrinsicGuessCam1Path_2"));

        _Cam1Intrinsicload_container_2->addWidget(_intrinsicGuessCam1Path_2);

        _intrinsicGuessCam1btnFileBrowse_2 = new QToolButton(scrollAreaWidgetContents_2);
        _intrinsicGuessCam1btnFileBrowse_2->setObjectName(QString::fromUtf8("_intrinsicGuessCam1btnFileBrowse_2"));

        _Cam1Intrinsicload_container_2->addWidget(_intrinsicGuessCam1btnFileBrowse_2);


        verticalLayout_15->addLayout(_Cam1Intrinsicload_container_2);

        tabWidget_3 = new QTabWidget(scrollAreaWidgetContents_2);
        tabWidget_3->setObjectName(QString::fromUtf8("tabWidget_3"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        gridLayout_6 = new QGridLayout(tab);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        groupBoxDetectCorners = new QGroupBox(tab);
        groupBoxDetectCorners->setObjectName(QString::fromUtf8("groupBoxDetectCorners"));
        sizePolicy6.setHeightForWidth(groupBoxDetectCorners->sizePolicy().hasHeightForWidth());
        groupBoxDetectCorners->setSizePolicy(sizePolicy6);
        gridLayout_4 = new QGridLayout(groupBoxDetectCorners);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        splitter = new QSplitter(groupBoxDetectCorners);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        label_38 = new QLabel(splitter);
        label_38->setObjectName(QString::fromUtf8("label_38"));
        splitter->addWidget(label_38);
        _annotationType_Combo_3 = new QComboBox(splitter);
        _annotationType_Combo_3->setObjectName(QString::fromUtf8("_annotationType_Combo_3"));
        splitter->addWidget(_annotationType_Combo_3);

        gridLayout_4->addWidget(splitter, 0, 0, 1, 1);

        _previewCorner_CB_2 = new QCheckBox(groupBoxDetectCorners);
        _previewCorner_CB_2->setObjectName(QString::fromUtf8("_previewCorner_CB_2"));
        _previewCorner_CB_2->setChecked(true);

        gridLayout_4->addWidget(_previewCorner_CB_2, 1, 0, 1, 1);

        _normalizeImg_CB_2 = new QCheckBox(groupBoxDetectCorners);
        _normalizeImg_CB_2->setObjectName(QString::fromUtf8("_normalizeImg_CB_2"));
        _normalizeImg_CB_2->setChecked(true);

        gridLayout_4->addWidget(_normalizeImg_CB_2, 2, 0, 1, 1);

        _adaptiveThres_CB_2 = new QCheckBox(groupBoxDetectCorners);
        _adaptiveThres_CB_2->setObjectName(QString::fromUtf8("_adaptiveThres_CB_2"));
        _adaptiveThres_CB_2->setChecked(true);

        gridLayout_4->addWidget(_adaptiveThres_CB_2, 3, 0, 1, 1);

        _preCornerAnalysis_CB_2 = new QCheckBox(groupBoxDetectCorners);
        _preCornerAnalysis_CB_2->setObjectName(QString::fromUtf8("_preCornerAnalysis_CB_2"));
        _preCornerAnalysis_CB_2->setChecked(true);

        gridLayout_4->addWidget(_preCornerAnalysis_CB_2, 4, 0, 1, 1);

        _filterQuads_CB_2 = new QCheckBox(groupBoxDetectCorners);
        _filterQuads_CB_2->setObjectName(QString::fromUtf8("_filterQuads_CB_2"));

        gridLayout_4->addWidget(_filterQuads_CB_2, 5, 0, 1, 1);

        _itc_IterativeTreminationFlags_Corner_BOX_2 = new QGroupBox(groupBoxDetectCorners);
        _itc_IterativeTreminationFlags_Corner_BOX_2->setObjectName(QString::fromUtf8("_itc_IterativeTreminationFlags_Corner_BOX_2"));
        _itc_IterativeTreminationFlags_Corner_BOX_2->setCheckable(true);
        verticalLayout_17 = new QVBoxLayout(_itc_IterativeTreminationFlags_Corner_BOX_2);
        verticalLayout_17->setObjectName(QString::fromUtf8("verticalLayout_17"));
        label_39 = new QLabel(_itc_IterativeTreminationFlags_Corner_BOX_2);
        label_39->setObjectName(QString::fromUtf8("label_39"));

        verticalLayout_17->addWidget(label_39);

        horizontalLayout_61 = new QHBoxLayout();
        horizontalLayout_61->setObjectName(QString::fromUtf8("horizontalLayout_61"));
        _itc_iterations_Corner_checkBox_3 = new QCheckBox(_itc_IterativeTreminationFlags_Corner_BOX_2);
        _itc_iterations_Corner_checkBox_3->setObjectName(QString::fromUtf8("_itc_iterations_Corner_checkBox_3"));
        _itc_iterations_Corner_checkBox_3->setChecked(true);

        horizontalLayout_61->addWidget(_itc_iterations_Corner_checkBox_3);

        horizontalSpacer_12 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_61->addItem(horizontalSpacer_12);

        _itc_max_Iter_Corner_LineEdit_2 = new QLineEdit(_itc_IterativeTreminationFlags_Corner_BOX_2);
        _itc_max_Iter_Corner_LineEdit_2->setObjectName(QString::fromUtf8("_itc_max_Iter_Corner_LineEdit_2"));

        horizontalLayout_61->addWidget(_itc_max_Iter_Corner_LineEdit_2);


        verticalLayout_17->addLayout(horizontalLayout_61);

        horizontalLayout_62 = new QHBoxLayout();
        horizontalLayout_62->setObjectName(QString::fromUtf8("horizontalLayout_62"));
        _itc_epsilon_Corner_checkBox_4 = new QCheckBox(_itc_IterativeTreminationFlags_Corner_BOX_2);
        _itc_epsilon_Corner_checkBox_4->setObjectName(QString::fromUtf8("_itc_epsilon_Corner_checkBox_4"));
        _itc_epsilon_Corner_checkBox_4->setChecked(true);

        horizontalLayout_62->addWidget(_itc_epsilon_Corner_checkBox_4);

        horizontalSpacer_13 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_62->addItem(horizontalSpacer_13);

        _itc_epsilon_Corner_LineEdit_3 = new QLineEdit(_itc_IterativeTreminationFlags_Corner_BOX_2);
        _itc_epsilon_Corner_LineEdit_3->setObjectName(QString::fromUtf8("_itc_epsilon_Corner_LineEdit_3"));

        horizontalLayout_62->addWidget(_itc_epsilon_Corner_LineEdit_3);


        verticalLayout_17->addLayout(horizontalLayout_62);

        verticalSpacer_6 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_17->addItem(verticalSpacer_6);


        gridLayout_4->addWidget(_itc_IterativeTreminationFlags_Corner_BOX_2, 6, 0, 1, 1);


        gridLayout_6->addWidget(groupBoxDetectCorners, 0, 0, 1, 1);

        tabWidget_3->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        groupBoxRobotPoseOnline = new QGroupBox(tab_2);
        groupBoxRobotPoseOnline->setObjectName(QString::fromUtf8("groupBoxRobotPoseOnline"));
        groupBoxRobotPoseOnline->setGeometry(QRect(10, 10, 421, 241));
        checkSaveLiveRobotPoses = new QCheckBox(groupBoxRobotPoseOnline);
        checkSaveLiveRobotPoses->setObjectName(QString::fromUtf8("checkSaveLiveRobotPoses"));
        checkSaveLiveRobotPoses->setGeometry(QRect(10, 190, 381, 22));
        checkSaveLiveRobotPoses->setChecked(true);
        layoutWidget1 = new QWidget(groupBoxRobotPoseOnline);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(0, 30, 421, 29));
        horizontalLayout_58 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_58->setObjectName(QString::fromUtf8("horizontalLayout_58"));
        horizontalLayout_58->setContentsMargins(0, 0, 0, 0);
        label_36 = new QLabel(layoutWidget1);
        label_36->setObjectName(QString::fromUtf8("label_36"));

        horizontalLayout_58->addWidget(label_36);

        comboAngleType = new QComboBox(layoutWidget1);
        comboAngleType->setObjectName(QString::fromUtf8("comboAngleType"));

        horizontalLayout_58->addWidget(comboAngleType);

        _RobotPosePathlabel_2 = new QLabel(groupBoxRobotPoseOnline);
        _RobotPosePathlabel_2->setObjectName(QString::fromUtf8("_RobotPosePathlabel_2"));
        _RobotPosePathlabel_2->setGeometry(QRect(0, 90, 191, 27));
        QFont font1;
        font1.setBold(false);
        font1.setItalic(false);
        font1.setWeight(50);
        _RobotPosePathlabel_2->setFont(font1);
        layoutWidget_6 = new QWidget(groupBoxRobotPoseOnline);
        layoutWidget_6->setObjectName(QString::fromUtf8("layoutWidget_6"));
        layoutWidget_6->setGeometry(QRect(0, 120, 421, 29));
        _Cam1Intrinsicload_container_5 = new QHBoxLayout(layoutWidget_6);
        _Cam1Intrinsicload_container_5->setObjectName(QString::fromUtf8("_Cam1Intrinsicload_container_5"));
        _Cam1Intrinsicload_container_5->setContentsMargins(0, 0, 0, 0);
        _RobotPoseSavePath = new QLineEdit(layoutWidget_6);
        _RobotPoseSavePath->setObjectName(QString::fromUtf8("_RobotPoseSavePath"));

        _Cam1Intrinsicload_container_5->addWidget(_RobotPoseSavePath);

        _RobotSavePathbtnFileBrowse_3 = new QToolButton(layoutWidget_6);
        _RobotSavePathbtnFileBrowse_3->setObjectName(QString::fromUtf8("_RobotSavePathbtnFileBrowse_3"));

        _Cam1Intrinsicload_container_5->addWidget(_RobotSavePathbtnFileBrowse_3);

        checkDisplayLiveRobotPoses = new QCheckBox(groupBoxRobotPoseOnline);
        checkDisplayLiveRobotPoses->setObjectName(QString::fromUtf8("checkDisplayLiveRobotPoses"));
        checkDisplayLiveRobotPoses->setGeometry(QRect(10, 220, 381, 22));
        label_37 = new QLabel(groupBoxRobotPoseOnline);
        label_37->setObjectName(QString::fromUtf8("label_37"));
        label_37->setGeometry(QRect(0, 70, 121, 17));
        label_37->setFont(font);
        labelRobotPoseCount = new QLabel(groupBoxRobotPoseOnline);
        labelRobotPoseCount->setObjectName(QString::fromUtf8("labelRobotPoseCount"));
        labelRobotPoseCount->setGeometry(QRect(125, 70, 71, 16));
        labelRobotPoseCount->setFont(font);
        layoutWidget2 = new QWidget(groupBoxRobotPoseOnline);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(0, 160, 421, 29));
        horizontalLayout_60 = new QHBoxLayout(layoutWidget2);
        horizontalLayout_60->setObjectName(QString::fromUtf8("horizontalLayout_60"));
        horizontalLayout_60->setContentsMargins(0, 0, 0, 0);
        _btnSaveRobotPoses = new QPushButton(layoutWidget2);
        _btnSaveRobotPoses->setObjectName(QString::fromUtf8("_btnSaveRobotPoses"));

        horizontalLayout_60->addWidget(_btnSaveRobotPoses);

        _btnClearRobotPoses = new QPushButton(layoutWidget2);
        _btnClearRobotPoses->setObjectName(QString::fromUtf8("_btnClearRobotPoses"));

        horizontalLayout_60->addWidget(_btnClearRobotPoses);

        groupBoxLoadRobotData = new QGroupBox(tab_2);
        groupBoxLoadRobotData->setObjectName(QString::fromUtf8("groupBoxLoadRobotData"));
        groupBoxLoadRobotData->setGeometry(QRect(10, 260, 431, 101));
        layoutWidget_5 = new QWidget(groupBoxLoadRobotData);
        layoutWidget_5->setObjectName(QString::fromUtf8("layoutWidget_5"));
        layoutWidget_5->setGeometry(QRect(0, 20, 421, 29));
        _Cam1Intrinsicload_container_3 = new QHBoxLayout(layoutWidget_5);
        _Cam1Intrinsicload_container_3->setObjectName(QString::fromUtf8("_Cam1Intrinsicload_container_3"));
        _Cam1Intrinsicload_container_3->setContentsMargins(0, 0, 0, 0);
        _RobotPosePath = new QLineEdit(layoutWidget_5);
        _RobotPosePath->setObjectName(QString::fromUtf8("_RobotPosePath"));

        _Cam1Intrinsicload_container_3->addWidget(_RobotPosePath);

        _RobotPathbtnFileBrowse = new QToolButton(layoutWidget_5);
        _RobotPathbtnFileBrowse->setObjectName(QString::fromUtf8("_RobotPathbtnFileBrowse"));

        _Cam1Intrinsicload_container_3->addWidget(_RobotPathbtnFileBrowse);

        _intrinsicReadFromFileBtnR_2 = new QPushButton(groupBoxLoadRobotData);
        _intrinsicReadFromFileBtnR_2->setObjectName(QString::fromUtf8("_intrinsicReadFromFileBtnR_2"));
        _intrinsicReadFromFileBtnR_2->setGeometry(QRect(0, 60, 391, 27));
        tabWidget_3->addTab(tab_2, QString());

        verticalLayout_15->addWidget(tabWidget_3);

        scrollArea_2->setWidget(scrollAreaWidgetContents_2);

        gridLayout_5->addWidget(scrollArea_2, 0, 0, 1, 1);

        horizontalLayout_59 = new QHBoxLayout();
        horizontalLayout_59->setObjectName(QString::fromUtf8("horizontalLayout_59"));
        _doSaveResultsBtn_2 = new QPushButton(tabHandEye);
        _doSaveResultsBtn_2->setObjectName(QString::fromUtf8("_doSaveResultsBtn_2"));
        sizePolicy8.setHeightForWidth(_doSaveResultsBtn_2->sizePolicy().hasHeightForWidth());
        _doSaveResultsBtn_2->setSizePolicy(sizePolicy8);

        horizontalLayout_59->addWidget(_doSaveResultsBtn_2);

        _clearAllData_2 = new QPushButton(tabHandEye);
        _clearAllData_2->setObjectName(QString::fromUtf8("_clearAllData_2"));
        sizePolicy8.setHeightForWidth(_clearAllData_2->sizePolicy().hasHeightForWidth());
        _clearAllData_2->setSizePolicy(sizePolicy8);

        horizontalLayout_59->addWidget(_clearAllData_2);

        _commitROS_2 = new QPushButton(tabHandEye);
        _commitROS_2->setObjectName(QString::fromUtf8("_commitROS_2"));
        sizePolicy8.setHeightForWidth(_commitROS_2->sizePolicy().hasHeightForWidth());
        _commitROS_2->setSizePolicy(sizePolicy8);

        horizontalLayout_59->addWidget(_commitROS_2);

        _cancelBtn_2 = new QPushButton(tabHandEye);
        _cancelBtn_2->setObjectName(QString::fromUtf8("_cancelBtn_2"));
        sizePolicy8.setHeightForWidth(_cancelBtn_2->sizePolicy().hasHeightForWidth());
        _cancelBtn_2->setSizePolicy(sizePolicy8);

        horizontalLayout_59->addWidget(_cancelBtn_2);


        gridLayout_5->addLayout(horizontalLayout_59, 1, 0, 1, 1);

        horizontalLayout_56 = new QHBoxLayout();
        horizontalLayout_56->setObjectName(QString::fromUtf8("horizontalLayout_56"));
        _doDetectCornersBtn_2 = new QPushButton(tabHandEye);
        _doDetectCornersBtn_2->setObjectName(QString::fromUtf8("_doDetectCornersBtn_2"));
        sizePolicy8.setHeightForWidth(_doDetectCornersBtn_2->sizePolicy().hasHeightForWidth());
        _doDetectCornersBtn_2->setSizePolicy(sizePolicy8);

        horizontalLayout_56->addWidget(_doDetectCornersBtn_2);

        _doComputeCamPoseBtn_2 = new QPushButton(tabHandEye);
        _doComputeCamPoseBtn_2->setObjectName(QString::fromUtf8("_doComputeCamPoseBtn_2"));
        sizePolicy8.setHeightForWidth(_doComputeCamPoseBtn_2->sizePolicy().hasHeightForWidth());
        _doComputeCamPoseBtn_2->setSizePolicy(sizePolicy8);

        horizontalLayout_56->addWidget(_doComputeCamPoseBtn_2);

        _doCalibrateBtn_2 = new QPushButton(tabHandEye);
        _doCalibrateBtn_2->setObjectName(QString::fromUtf8("_doCalibrateBtn_2"));
        sizePolicy8.setHeightForWidth(_doCalibrateBtn_2->sizePolicy().hasHeightForWidth());
        _doCalibrateBtn_2->setSizePolicy(sizePolicy8);

        horizontalLayout_56->addWidget(_doCalibrateBtn_2);

        _doAllBtn_2 = new QPushButton(tabHandEye);
        _doAllBtn_2->setObjectName(QString::fromUtf8("_doAllBtn_2"));
        _doAllBtn_2->setEnabled(true);
        sizePolicy8.setHeightForWidth(_doAllBtn_2->sizePolicy().hasHeightForWidth());
        _doAllBtn_2->setSizePolicy(sizePolicy8);

        horizontalLayout_56->addWidget(_doAllBtn_2);


        gridLayout_5->addLayout(horizontalLayout_56, 2, 0, 1, 1);

        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/images/robot.jpg"), QSize(), QIcon::Normal, QIcon::Off);
        tabWidget_2->addTab(tabHandEye, icon2, QString());

        verticalLayout->addWidget(tabWidget_2);


        horizontalLayout_7->addWidget(verticalGroupBox);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setSizeConstraint(QLayout::SetDefaultConstraint);
        horizontalLayout_imgPreview = new QHBoxLayout();
        horizontalLayout_imgPreview->setObjectName(QString::fromUtf8("horizontalLayout_imgPreview"));
        horizontalLayout_imgPreview->setSizeConstraint(QLayout::SetFixedSize);
        horizontalLayout_imgPreview->setContentsMargins(-1, 5, -1, -1);
        verticalLayout_previewL = new QVBoxLayout();
        verticalLayout_previewL->setObjectName(QString::fromUtf8("verticalLayout_previewL"));
        imgLabelL = new QLabel(centralwidget);
        imgLabelL->setObjectName(QString::fromUtf8("imgLabelL"));
        QSizePolicy sizePolicy9(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy9.setHorizontalStretch(0);
        sizePolicy9.setVerticalStretch(255);
        sizePolicy9.setHeightForWidth(imgLabelL->sizePolicy().hasHeightForWidth());
        imgLabelL->setSizePolicy(sizePolicy9);
        imgLabelL->setMinimumSize(QSize(320, 240));
        QFont font2;
        font2.setPointSize(10);
        font2.setBold(true);
        font2.setWeight(75);
        imgLabelL->setFont(font2);

        verticalLayout_previewL->addWidget(imgLabelL);


        horizontalLayout_imgPreview->addLayout(verticalLayout_previewL);

        verticalLayout_previewR = new QVBoxLayout();
        verticalLayout_previewR->setObjectName(QString::fromUtf8("verticalLayout_previewR"));
        imgLabelR = new QLabel(centralwidget);
        imgLabelR->setObjectName(QString::fromUtf8("imgLabelR"));
        sizePolicy9.setHeightForWidth(imgLabelR->sizePolicy().hasHeightForWidth());
        imgLabelR->setSizePolicy(sizePolicy9);
        imgLabelR->setMinimumSize(QSize(320, 240));
        imgLabelR->setFont(font2);
        imgLabelR->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        verticalLayout_previewR->addWidget(imgLabelR);


        horizontalLayout_imgPreview->addLayout(verticalLayout_previewR);


        verticalLayout_2->addLayout(horizontalLayout_imgPreview);

        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        horizontalLayout_55 = new QHBoxLayout(groupBox);
        horizontalLayout_55->setObjectName(QString::fromUtf8("horizontalLayout_55"));
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_55->addWidget(label_4);

        comboBoxImageCurrent = new QComboBox(groupBox);
        comboBoxImageCurrent->setObjectName(QString::fromUtf8("comboBoxImageCurrent"));

        horizontalLayout_55->addWidget(comboBoxImageCurrent);

        pushButtonImageDelete = new QPushButton(groupBox);
        pushButtonImageDelete->setObjectName(QString::fromUtf8("pushButtonImageDelete"));

        horizontalLayout_55->addWidget(pushButtonImageDelete);


        verticalLayout_2->addWidget(groupBox);

        liveStreamBox = new QGroupBox(centralwidget);
        liveStreamBox->setObjectName(QString::fromUtf8("liveStreamBox"));
        gridLayout = new QGridLayout(liveStreamBox);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        lineEditImgTopicR = new QLineEdit(liveStreamBox);
        lineEditImgTopicR->setObjectName(QString::fromUtf8("lineEditImgTopicR"));

        gridLayout->addWidget(lineEditImgTopicR, 6, 2, 1, 1);

        pushButtonStreamSaveImage = new QPushButton(liveStreamBox);
        pushButtonStreamSaveImage->setObjectName(QString::fromUtf8("pushButtonStreamSaveImage"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/images/save.png"), QSize(), QIcon::Normal, QIcon::Off);
        pushButtonStreamSaveImage->setIcon(icon3);

        gridLayout->addWidget(pushButtonStreamSaveImage, 7, 2, 1, 1);

        pushButtonStreamStop = new QPushButton(liveStreamBox);
        pushButtonStreamStop->setObjectName(QString::fromUtf8("pushButtonStreamStop"));
        pushButtonStreamStop->setEnabled(false);
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/images/stop-red.png"), QSize(), QIcon::Normal, QIcon::Off);
        pushButtonStreamStop->setIcon(icon4);

        gridLayout->addWidget(pushButtonStreamStop, 7, 1, 1, 1);

        pushButtonStreamStart = new QPushButton(liveStreamBox);
        pushButtonStreamStart->setObjectName(QString::fromUtf8("pushButtonStreamStart"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/images/play1.png"), QSize(), QIcon::Normal, QIcon::Off);
        pushButtonStreamStart->setIcon(icon5);

        gridLayout->addWidget(pushButtonStreamStart, 7, 0, 1, 1);

        label_2 = new QLabel(liveStreamBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 6, 0, 1, 1);

        lineEditImgTopicL = new QLineEdit(liveStreamBox);
        lineEditImgTopicL->setObjectName(QString::fromUtf8("lineEditImgTopicL"));

        gridLayout->addWidget(lineEditImgTopicL, 6, 1, 1, 1);

        horizontalLayout_57 = new QHBoxLayout();
        horizontalLayout_57->setObjectName(QString::fromUtf8("horizontalLayout_57"));
        labelRightTopic = new QLabel(liveStreamBox);
        labelRightTopic->setObjectName(QString::fromUtf8("labelRightTopic"));
        labelRightTopic->setFont(font);

        horizontalLayout_57->addWidget(labelRightTopic);

        checkBoxIsInverted = new QCheckBox(liveStreamBox);
        checkBoxIsInverted->setObjectName(QString::fromUtf8("checkBoxIsInverted"));
        QSizePolicy sizePolicy10(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy10.setHorizontalStretch(0);
        sizePolicy10.setVerticalStretch(0);
        sizePolicy10.setHeightForWidth(checkBoxIsInverted->sizePolicy().hasHeightForWidth());
        checkBoxIsInverted->setSizePolicy(sizePolicy10);
        checkBoxIsInverted->setChecked(false);

        horizontalLayout_57->addWidget(checkBoxIsInverted);


        gridLayout->addLayout(horizontalLayout_57, 3, 2, 2, 1);

        labelLeftTopic = new QLabel(liveStreamBox);
        labelLeftTopic->setObjectName(QString::fromUtf8("labelLeftTopic"));
        labelLeftTopic->setFont(font);

        gridLayout->addWidget(labelLeftTopic, 4, 1, 1, 1);


        verticalLayout_2->addWidget(liveStreamBox);

        _console = new QTextBrowser(centralwidget);
        _console->setObjectName(QString::fromUtf8("_console"));
        sizePolicy7.setHeightForWidth(_console->sizePolicy().hasHeightForWidth());
        _console->setSizePolicy(sizePolicy7);

        verticalLayout_2->addWidget(_console);

        _clearConsoleBtn = new QPushButton(centralwidget);
        _clearConsoleBtn->setObjectName(QString::fromUtf8("_clearConsoleBtn"));
        sizePolicy10.setHeightForWidth(_clearConsoleBtn->sizePolicy().hasHeightForWidth());
        _clearConsoleBtn->setSizePolicy(sizePolicy10);

        verticalLayout_2->addWidget(_clearConsoleBtn);


        horizontalLayout_7->addLayout(verticalLayout_2);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1180, 25));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);

        menubar->addAction(menu_File->menuAction());
        menu_File->addSeparator();
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));

        tabWidget_2->setCurrentIndex(1);
        tabWidget->setCurrentIndex(1);
        _inspectCamTabs->setCurrentIndex(1);
        tabWidget_3->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "CalTI", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindowDesign", "Select calibration type", 0, QApplication::UnicodeUTF8));
        labelPreviewSize->setText(QApplication::translate("MainWindowDesign", "Preview size", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindowDesign", "Image filter and directory path", 0, QApplication::UnicodeUTF8));
        _fileFilterL_label->setText(QApplication::translate("MainWindowDesign", "Left", 0, QApplication::UnicodeUTF8));
        btnFileBrowseL->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        _fileFilterR_label->setText(QApplication::translate("MainWindowDesign", "Right", 0, QApplication::UnicodeUTF8));
        btnFileBrowseR->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("MainWindowDesign", "Calibration object", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindowDesign", "Select board type:", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindowDesign", "Size of board [ny,nx]:", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindowDesign", "Physical board size [mm]:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        tabWidget->setToolTip(QString());
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        tabCornerFlags->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        label_8->setText(QApplication::translate("MainWindowDesign", "Annotation type", 0, QApplication::UnicodeUTF8));
        _previewCorner_CB->setText(QApplication::translate("MainWindowDesign", "Preview corner detection", 0, QApplication::UnicodeUTF8));
        _normalizeImg_CB->setText(QApplication::translate("MainWindowDesign", "Normalize image", 0, QApplication::UnicodeUTF8));
        _adaptiveThres_CB->setText(QApplication::translate("MainWindowDesign", "Use adaptive threshold", 0, QApplication::UnicodeUTF8));
        _preCornerAnalysis_CB->setText(QApplication::translate("MainWindowDesign", "Use pre corner analysis", 0, QApplication::UnicodeUTF8));
        _filterQuads_CB->setText(QApplication::translate("MainWindowDesign", "Filter quads", 0, QApplication::UnicodeUTF8));
        _itc_IterativeTreminationFlags_Corner_BOX->setTitle(QApplication::translate("MainWindowDesign", "Iterative termination criteria for corner detection", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("MainWindowDesign", "Iterative break on (which ever comes first):", 0, QApplication::UnicodeUTF8));
        _itc_iterations_Corner_checkBox->setText(QApplication::translate("MainWindowDesign", "Number of iterations", 0, QApplication::UnicodeUTF8));
        _itc_max_Iter_Corner_LineEdit_->setText(QApplication::translate("MainWindowDesign", "100", 0, QApplication::UnicodeUTF8));
        _itc_epsilon_Corner_checkBox_2->setText(QApplication::translate("MainWindowDesign", "Minimum accuracy", 0, QApplication::UnicodeUTF8));
        _itc_epsilon_Corner_LineEdit->setText(QApplication::translate("MainWindowDesign", "1e-5", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tabCornerFlags), QApplication::translate("MainWindowDesign", "Corner Detection", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        tabCalFlags->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        _CalFlags_groupBox->setTitle(QApplication::translate("MainWindowDesign", "Camera calibration settings", 0, QApplication::UnicodeUTF8));
        _useRationalModel_CB->setText(QApplication::translate("MainWindowDesign", "Use rational model", 0, QApplication::UnicodeUTF8));
        _useIntrinsicGuess_CB->setText(QApplication::translate("MainWindowDesign", "Use intrinsic guess", 0, QApplication::UnicodeUTF8));
        _intrinsicGuessCam1label->setText(QApplication::translate("MainWindowDesign", "Camera #1", 0, QApplication::UnicodeUTF8));
        _intrinsicGuessCam1Path->setText(QApplication::translate("MainWindowDesign", "G:\\pictures\\temp\\1\\intrinsic.yml", 0, QApplication::UnicodeUTF8));
        _intrinsicGuessCam1btnFileBrowse->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        _intrinsicGuessCam2label->setText(QApplication::translate("MainWindowDesign", "Camera #2", 0, QApplication::UnicodeUTF8));
        _intrinsicGuessCam2Path->setText(QApplication::translate("MainWindowDesign", "G:\\pictures\\temp\\2\\intrinsic.yml", 0, QApplication::UnicodeUTF8));
        _intrinsicGuessCam2btnFileBrowse->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        groupBox_inspectIntrinsic->setTitle(QApplication::translate("MainWindowDesign", "Inspect intrinsic values", 0, QApplication::UnicodeUTF8));
        _intrinsicReadFromFileBtnL->setText(QApplication::translate("MainWindowDesign", "Read values from file", 0, QApplication::UnicodeUTF8));
        _intrinsicClearValsBtnL->setText(QApplication::translate("MainWindowDesign", "Clear values", 0, QApplication::UnicodeUTF8));
        _fixPrincipalPoint_CB_CL->setText(QApplication::translate("MainWindowDesign", "Fix the principal point", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("MainWindowDesign", "x:", 0, QApplication::UnicodeUTF8));
        _fixPrincipalPoint_VALx_CL->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("MainWindowDesign", "y:", 0, QApplication::UnicodeUTF8));
        _fixPrincipalPoint_VALy_CL->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fixFocalLength_CB_CL->setText(QApplication::translate("MainWindowDesign", "Fix focal length", 0, QApplication::UnicodeUTF8));
        _fixAspectRatio_CB_CL->setText(QApplication::translate("MainWindowDesign", "Fix aspect ratio", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("MainWindowDesign", "fx:", 0, QApplication::UnicodeUTF8));
        _fixFocalLength_VALx_CL->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("MainWindowDesign", "fy:", 0, QApplication::UnicodeUTF8));
        _fixFocalLength_VALy_CL->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("MainWindowDesign", "Distortion parameters", 0, QApplication::UnicodeUTF8));
        _fixZeroTangentDistortion_CB_CL->setText(QApplication::translate("MainWindowDesign", "Fix zero tangent distortion parameter", 0, QApplication::UnicodeUTF8));
        label_24->setText(QApplication::translate("MainWindowDesign", "P1:", 0, QApplication::UnicodeUTF8));
        _fix_P1_VAL_CL->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        label_25->setText(QApplication::translate("MainWindowDesign", "P2:", 0, QApplication::UnicodeUTF8));
        _fix_P2_VAL_CL->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fix_K1_CB_CL->setText(QApplication::translate("MainWindowDesign", "Fix K1", 0, QApplication::UnicodeUTF8));
        _fix_K1_VAL_CL->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fix_K2_CB_CL->setText(QApplication::translate("MainWindowDesign", "Fix K2", 0, QApplication::UnicodeUTF8));
        _fix_K2_VAL_CL->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fix_K3_CB_CL->setText(QApplication::translate("MainWindowDesign", "Fix K3", 0, QApplication::UnicodeUTF8));
        _fix_K3_VAL_CL->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fix_K4_CB_CL->setText(QApplication::translate("MainWindowDesign", "Fix K4", 0, QApplication::UnicodeUTF8));
        _fix_K4_VAL_CL->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fix_K5_CB_CL->setText(QApplication::translate("MainWindowDesign", "Fix K5", 0, QApplication::UnicodeUTF8));
        _fix_K5_VAL_CL->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fix_K6_CB_CL->setText(QApplication::translate("MainWindowDesign", "Fix K6", 0, QApplication::UnicodeUTF8));
        _fix_K6_VAL_CL->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _inspectCamTabs->setTabText(_inspectCamTabs->indexOf(_inspectIntrinsicCamera1Tab), QApplication::translate("MainWindowDesign", "Camera #1", 0, QApplication::UnicodeUTF8));
        _intrinsicReadFromFileBtnR->setText(QApplication::translate("MainWindowDesign", "Read values from file", 0, QApplication::UnicodeUTF8));
        _intrinsicClearValsBtnR->setText(QApplication::translate("MainWindowDesign", "Clear values", 0, QApplication::UnicodeUTF8));
        _fixPrincipalPoint_CB_CR->setText(QApplication::translate("MainWindowDesign", "Fix the principal point", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("MainWindowDesign", "x:", 0, QApplication::UnicodeUTF8));
        _fixPrincipalPoint_VALx_CR->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("MainWindowDesign", "y:", 0, QApplication::UnicodeUTF8));
        _fixPrincipalPoint_VALy_CR->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fixFocalLength_CB_CR->setText(QApplication::translate("MainWindowDesign", "Fix focal length", 0, QApplication::UnicodeUTF8));
        _fixAspectRatio_CB_CR->setText(QApplication::translate("MainWindowDesign", "Fix aspect ratio", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("MainWindowDesign", "fx:", 0, QApplication::UnicodeUTF8));
        _fixFocalLength_VALx_CR->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        label_19->setText(QApplication::translate("MainWindowDesign", "fy:", 0, QApplication::UnicodeUTF8));
        _fixFocalLength_VALy_CR->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("MainWindowDesign", "Distortion parameters", 0, QApplication::UnicodeUTF8));
        _fixZeroTangentDistortion_CB_CR->setText(QApplication::translate("MainWindowDesign", "Fix zero tangent distortion parameter", 0, QApplication::UnicodeUTF8));
        label_22->setText(QApplication::translate("MainWindowDesign", "P1:", 0, QApplication::UnicodeUTF8));
        _fix_P1_VAL_CR->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        label_23->setText(QApplication::translate("MainWindowDesign", "P2:", 0, QApplication::UnicodeUTF8));
        _fix_P2_VAL_CR->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fix_K1_CB_CR->setText(QApplication::translate("MainWindowDesign", "Fix K1", 0, QApplication::UnicodeUTF8));
        _fix_K1_VAL_CR->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fix_K2_CB_CR->setText(QApplication::translate("MainWindowDesign", "Fix K2", 0, QApplication::UnicodeUTF8));
        _fix_K2_VAL_CR->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fix_K3_CB_CR->setText(QApplication::translate("MainWindowDesign", "Fix K3", 0, QApplication::UnicodeUTF8));
        _fix_K3_VAL_CR->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fix_K4_CB_CR->setText(QApplication::translate("MainWindowDesign", "Fix K4", 0, QApplication::UnicodeUTF8));
        _fix_K4_VAL_CR->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fix_K5_CB_CR->setText(QApplication::translate("MainWindowDesign", "Fix K5", 0, QApplication::UnicodeUTF8));
        _fix_K5_VAL_CR->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _fix_K6_CB_CR->setText(QApplication::translate("MainWindowDesign", "Fix K6", 0, QApplication::UnicodeUTF8));
        _fix_K6_VAL_CR->setText(QApplication::translate("MainWindowDesign", "0.0", 0, QApplication::UnicodeUTF8));
        _inspectCamTabs->setTabText(_inspectCamTabs->indexOf(_inspectIntrinsicCamera2Tab), QApplication::translate("MainWindowDesign", "Camera #2", 0, QApplication::UnicodeUTF8));
        groupBox_StereoCalFlags->setTitle(QApplication::translate("MainWindowDesign", "Show stereo calibration flags", 0, QApplication::UnicodeUTF8));
        _fixIntrinsicStereo_CB->setText(QApplication::translate("MainWindowDesign", "Fix intrinsic", 0, QApplication::UnicodeUTF8));
        _sameFocalLengthStereo_CB->setText(QApplication::translate("MainWindowDesign", "Force focal length to be identical", 0, QApplication::UnicodeUTF8));
        _itc_IterativeTreminationFlags_Stereo_BOX->setTitle(QApplication::translate("MainWindowDesign", "Use iterative termination criteria", 0, QApplication::UnicodeUTF8));
        label_26->setText(QApplication::translate("MainWindowDesign", "Iterative break on (which ever comes first):", 0, QApplication::UnicodeUTF8));
        _itc_iterations_Stereo_checkBox->setText(QApplication::translate("MainWindowDesign", "Number of iterations", 0, QApplication::UnicodeUTF8));
        _itc_max_Iter_Stereo_LineEdit->setText(QApplication::translate("MainWindowDesign", "100", 0, QApplication::UnicodeUTF8));
        _itc_epsilon_Stereo_checkBox->setText(QApplication::translate("MainWindowDesign", "Minimum accuracy", 0, QApplication::UnicodeUTF8));
        _itc_epsilon_Stereo_LineEdit->setText(QApplication::translate("MainWindowDesign", "0.00005", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tabCalFlags), QApplication::translate("MainWindowDesign", "Calibration", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("MainWindowDesign", "Rectification flags", 0, QApplication::UnicodeUTF8));
        _rectAlgorithm_Label->setText(QApplication::translate("MainWindowDesign", "Rectification algorithm", 0, QApplication::UnicodeUTF8));
        _rectFundamentalLabel->setText(QApplication::translate("MainWindowDesign", "Fundamental matrix algorithm", 0, QApplication::UnicodeUTF8));
        _rectParam1_label->setText(QApplication::translate("MainWindowDesign", "Param1", 0, QApplication::UnicodeUTF8));
        _rectParam1_VAL->setText(QApplication::translate("MainWindowDesign", "3.0", 0, QApplication::UnicodeUTF8));
        _rectParam2_label->setText(QApplication::translate("MainWindowDesign", "Param2", 0, QApplication::UnicodeUTF8));
        _rectParam2_VAL->setText(QApplication::translate("MainWindowDesign", "0.99", 0, QApplication::UnicodeUTF8));
        _rectAlpha_CB->setText(QApplication::translate("MainWindowDesign", "Use custom alpha (scaling)", 0, QApplication::UnicodeUTF8));
        _zeroRectifyDisparity_CB->setText(QApplication::translate("MainWindowDesign", " Zero Rectification Disparity", 0, QApplication::UnicodeUTF8));
        _rectDisplayValidRect_CB->setText(QApplication::translate("MainWindowDesign", "Display valid rectification area", 0, QApplication::UnicodeUTF8));
        _displayRectification_CB->setText(QApplication::translate("MainWindowDesign", "Display rectification results", 0, QApplication::UnicodeUTF8));
        _saveRectification_CB->setText(QApplication::translate("MainWindowDesign", "Save rectification images", 0, QApplication::UnicodeUTF8));
        _rectAllImgsInFolder->setText(QApplication::translate("MainWindowDesign", "Rectify all images in src folder", 0, QApplication::UnicodeUTF8));
        _rectSavePathLeftLabel->setText(QApplication::translate("MainWindowDesign", "Save path left:", 0, QApplication::UnicodeUTF8));
        _rectificationSavePathTxtL->setText(QApplication::translate("MainWindowDesign", "G:\\pictures\\temp\\1\\Rectified", 0, QApplication::UnicodeUTF8));
        _rectificationSavePathBtnL->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        _rectSavePathRightLabel->setText(QApplication::translate("MainWindowDesign", "Save path right:", 0, QApplication::UnicodeUTF8));
        _rectificationSavePathTxtR->setText(QApplication::translate("MainWindowDesign", "G:\\pictures\\temp\\2\\Rectified", 0, QApplication::UnicodeUTF8));
        _rectificationSavePathBtnR->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tabRectFlags), QApplication::translate("MainWindowDesign", "Rectification", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("MainWindowDesign", "Intrinsic left:", 0, QApplication::UnicodeUTF8));
        btnFileBrowserNSIntrinsicLeft->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("MainWindowDesign", "Intrinsic right:", 0, QApplication::UnicodeUTF8));
        btnFileBrowserNSIntrinsicRight->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        label_27->setText(QApplication::translate("MainWindowDesign", "Image left:", 0, QApplication::UnicodeUTF8));
        btnFileBrowserNSLeftImage->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        label_28->setText(QApplication::translate("MainWindowDesign", "Image right:", 0, QApplication::UnicodeUTF8));
        btnFileBrowserNSRightImage->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        label_30->setText(QApplication::translate("MainWindowDesign", "Output left:", 0, QApplication::UnicodeUTF8));
        label_31->setText(QApplication::translate("MainWindowDesign", "Output right:", 0, QApplication::UnicodeUTF8));
        pushButtonNarrowCompute->setText(QApplication::translate("MainWindowDesign", "Compute extrinsics", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tabNarrowStereo), QApplication::translate("MainWindowDesign", "Narrow Stereo", 0, QApplication::UnicodeUTF8));
        _doSaveResultsBtn->setText(QApplication::translate("MainWindowDesign", "Save results", 0, QApplication::UnicodeUTF8));
        _clearAllData->setText(QApplication::translate("MainWindowDesign", "Clear all data", 0, QApplication::UnicodeUTF8));
        _commitROS->setText(QApplication::translate("MainWindowDesign", "Commit to ROS", 0, QApplication::UnicodeUTF8));
        _cancelBtn->setText(QApplication::translate("MainWindowDesign", "Cancel", 0, QApplication::UnicodeUTF8));
        _doDetectCornersBtn->setText(QApplication::translate("MainWindowDesign", "Detect corners", 0, QApplication::UnicodeUTF8));
        _doCalibrateBtn->setText(QApplication::translate("MainWindowDesign", "Calibrate", 0, QApplication::UnicodeUTF8));
        _doRectifyBtn->setText(QApplication::translate("MainWindowDesign", "Rectify", 0, QApplication::UnicodeUTF8));
        _doAllBtn->setText(QApplication::translate("MainWindowDesign", "Do all", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tabCameraCalib), QApplication::translate("MainWindowDesign", "Camera Calibration", 0, QApplication::UnicodeUTF8));
        label_29->setText(QApplication::translate("MainWindowDesign", "Select calibration type", 0, QApplication::UnicodeUTF8));
        labelPreviewSize_2->setText(QApplication::translate("MainWindowDesign", "Preview size", 0, QApplication::UnicodeUTF8));
        label_32->setText(QApplication::translate("MainWindowDesign", "Image filter and directory path", 0, QApplication::UnicodeUTF8));
        _fileFilterL_label_2->setText(QApplication::translate("MainWindowDesign", "Left", 0, QApplication::UnicodeUTF8));
        btnFileBrowseL_2->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("MainWindowDesign", "Calibration object", 0, QApplication::UnicodeUTF8));
        label_33->setText(QApplication::translate("MainWindowDesign", "Select board type:", 0, QApplication::UnicodeUTF8));
        label_34->setText(QApplication::translate("MainWindowDesign", "Size of board [ny,nx]:", 0, QApplication::UnicodeUTF8));
        label_35->setText(QApplication::translate("MainWindowDesign", "Physical board size [mm]:", 0, QApplication::UnicodeUTF8));
        labelPreviewSize_3->setText(QApplication::translate("MainWindowDesign", "Camera Intrinsic", 0, QApplication::UnicodeUTF8));
        _intrinsicGuessCam1label_2->setText(QApplication::translate("MainWindowDesign", "Camera #1", 0, QApplication::UnicodeUTF8));
        _intrinsicGuessCam1Path_2->setText(QApplication::translate("MainWindowDesign", "G:\\pictures\\temp\\1\\intrinsic.yml", 0, QApplication::UnicodeUTF8));
        _intrinsicGuessCam1btnFileBrowse_2->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        groupBoxDetectCorners->setTitle(QApplication::translate("MainWindowDesign", "Corner Detection", 0, QApplication::UnicodeUTF8));
        label_38->setText(QApplication::translate("MainWindowDesign", "Annotation type", 0, QApplication::UnicodeUTF8));
        _previewCorner_CB_2->setText(QApplication::translate("MainWindowDesign", "Preview corner detection", 0, QApplication::UnicodeUTF8));
        _normalizeImg_CB_2->setText(QApplication::translate("MainWindowDesign", "Normalize image", 0, QApplication::UnicodeUTF8));
        _adaptiveThres_CB_2->setText(QApplication::translate("MainWindowDesign", "Use adaptive threshold", 0, QApplication::UnicodeUTF8));
        _preCornerAnalysis_CB_2->setText(QApplication::translate("MainWindowDesign", "Use pre corner analysis", 0, QApplication::UnicodeUTF8));
        _filterQuads_CB_2->setText(QApplication::translate("MainWindowDesign", "Filter quads", 0, QApplication::UnicodeUTF8));
        _itc_IterativeTreminationFlags_Corner_BOX_2->setTitle(QApplication::translate("MainWindowDesign", "Iterative termination criteria for corner detection", 0, QApplication::UnicodeUTF8));
        label_39->setText(QApplication::translate("MainWindowDesign", "Iterative break on (which ever comes first):", 0, QApplication::UnicodeUTF8));
        _itc_iterations_Corner_checkBox_3->setText(QApplication::translate("MainWindowDesign", "Number of iterations", 0, QApplication::UnicodeUTF8));
        _itc_max_Iter_Corner_LineEdit_2->setText(QApplication::translate("MainWindowDesign", "100", 0, QApplication::UnicodeUTF8));
        _itc_epsilon_Corner_checkBox_4->setText(QApplication::translate("MainWindowDesign", "Minimum accuracy", 0, QApplication::UnicodeUTF8));
        _itc_epsilon_Corner_LineEdit_3->setText(QApplication::translate("MainWindowDesign", "1e-5", 0, QApplication::UnicodeUTF8));
        tabWidget_3->setTabText(tabWidget_3->indexOf(tab), QApplication::translate("MainWindowDesign", "Corner Detection", 0, QApplication::UnicodeUTF8));
        groupBoxRobotPoseOnline->setTitle(QApplication::translate("MainWindowDesign", "Online Robot Pose Recording", 0, QApplication::UnicodeUTF8));
        checkSaveLiveRobotPoses->setText(QApplication::translate("MainWindowDesign", "Save Robot Poses", 0, QApplication::UnicodeUTF8));
        label_36->setText(QApplication::translate("MainWindowDesign", "Select angle representation:", 0, QApplication::UnicodeUTF8));
        _RobotPosePathlabel_2->setText(QApplication::translate("MainWindowDesign", "Save robot poses at:", 0, QApplication::UnicodeUTF8));
        _RobotPoseSavePath->setText(QApplication::translate("MainWindowDesign", "G:\\pictures\\temp\\1\\intrinsic.yml", 0, QApplication::UnicodeUTF8));
        _RobotSavePathbtnFileBrowse_3->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        checkDisplayLiveRobotPoses->setText(QApplication::translate("MainWindowDesign", "Display Robot Poses", 0, QApplication::UnicodeUTF8));
        label_37->setText(QApplication::translate("MainWindowDesign", "Recorded poses:", 0, QApplication::UnicodeUTF8));
        labelRobotPoseCount->setText(QApplication::translate("MainWindowDesign", "0", 0, QApplication::UnicodeUTF8));
        _btnSaveRobotPoses->setText(QApplication::translate("MainWindowDesign", "Save", 0, QApplication::UnicodeUTF8));
        _btnClearRobotPoses->setText(QApplication::translate("MainWindowDesign", "Clear", 0, QApplication::UnicodeUTF8));
        groupBoxLoadRobotData->setTitle(QApplication::translate("MainWindowDesign", "Load Robot pose:", 0, QApplication::UnicodeUTF8));
        _RobotPosePath->setText(QApplication::translate("MainWindowDesign", "G:\\pictures\\temp\\1\\intrinsic.yml", 0, QApplication::UnicodeUTF8));
        _RobotPathbtnFileBrowse->setText(QApplication::translate("MainWindowDesign", "...", 0, QApplication::UnicodeUTF8));
        _intrinsicReadFromFileBtnR_2->setText(QApplication::translate("MainWindowDesign", "Display values from file", 0, QApplication::UnicodeUTF8));
        tabWidget_3->setTabText(tabWidget_3->indexOf(tab_2), QApplication::translate("MainWindowDesign", "Robot Pose Recording", 0, QApplication::UnicodeUTF8));
        _doSaveResultsBtn_2->setText(QApplication::translate("MainWindowDesign", "Save results", 0, QApplication::UnicodeUTF8));
        _clearAllData_2->setText(QApplication::translate("MainWindowDesign", "Clear all data", 0, QApplication::UnicodeUTF8));
        _commitROS_2->setText(QApplication::translate("MainWindowDesign", "Commit to ROS", 0, QApplication::UnicodeUTF8));
        _cancelBtn_2->setText(QApplication::translate("MainWindowDesign", "Cancel", 0, QApplication::UnicodeUTF8));
        _doDetectCornersBtn_2->setText(QApplication::translate("MainWindowDesign", "Detect corners", 0, QApplication::UnicodeUTF8));
        _doComputeCamPoseBtn_2->setText(QApplication::translate("MainWindowDesign", "Calc Poses", 0, QApplication::UnicodeUTF8));
        _doCalibrateBtn_2->setText(QApplication::translate("MainWindowDesign", "Calibrate", 0, QApplication::UnicodeUTF8));
        _doAllBtn_2->setText(QApplication::translate("MainWindowDesign", "Do all", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tabHandEye), QApplication::translate("MainWindowDesign", "Hand/Eye Calibration", 0, QApplication::UnicodeUTF8));
        imgLabelL->setText(QApplication::translate("MainWindowDesign", "No image loaded yet", 0, QApplication::UnicodeUTF8));
        imgLabelR->setText(QApplication::translate("MainWindowDesign", "No image loaded yet", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindowDesign", "Current images", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindowDesign", "Select image:", 0, QApplication::UnicodeUTF8));
        pushButtonImageDelete->setText(QApplication::translate("MainWindowDesign", "Delete image", 0, QApplication::UnicodeUTF8));
        liveStreamBox->setTitle(QApplication::translate("MainWindowDesign", "Live stream", 0, QApplication::UnicodeUTF8));
        pushButtonStreamSaveImage->setText(QApplication::translate("MainWindowDesign", "Save image", 0, QApplication::UnicodeUTF8));
        pushButtonStreamStop->setText(QApplication::translate("MainWindowDesign", "Stop stream", 0, QApplication::UnicodeUTF8));
        pushButtonStreamStart->setText(QApplication::translate("MainWindowDesign", "Start stream", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindowDesign", "Image topics:", 0, QApplication::UnicodeUTF8));
        labelRightTopic->setText(QApplication::translate("MainWindowDesign", "Right Topic:", 0, QApplication::UnicodeUTF8));
        checkBoxIsInverted->setText(QApplication::translate("MainWindowDesign", "Is Inverted", 0, QApplication::UnicodeUTF8));
        labelLeftTopic->setText(QApplication::translate("MainWindowDesign", "Left Topic:", 0, QApplication::UnicodeUTF8));
        _clearConsoleBtn->setText(QApplication::translate("MainWindowDesign", "Clear console", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H

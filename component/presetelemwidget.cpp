#include "presetelemwidget.h"
#include"mainwindow.h"
#include<QGridLayout>
#include<QLabel>
#include"geometry/globes.h"
#include<QMessageBox>
#include<QGroupBox>
#include<QVector4D>

PresetElemWidget::PresetElemWidget(QWidget *parent)
    : QWidget{parent}
{
    //保留当前窗口标志中的所有特性，并设置为具有对话框特性的窗口
    setWindowFlags(windowFlags()|Qt::Dialog);
    resize(480,320);
    m_pMainWin =(MainWindow*)parent; //强制类型转换

    // 创建tab widget
    tabWidget = new QTabWidget(this);

    // 创建第一个tab的内容（点）
    QWidget *firstTab = new QWidget();
    QGridLayout *firstLayout = new QGridLayout(firstTab);
    // 创建标签和文本框
    QLabel *pointLabelX = new QLabel("X:");
    pointEditX = new QLineEdit();
    pointEditX->setText("0");
    pointEditX->setMaximumWidth(150);
    QLabel *pointLabelY = new QLabel("Y:");
    pointEditY = new QLineEdit();
    pointEditY->setText("0");
    pointEditY->setMaximumWidth(150);
    QLabel *pointLabelZ = new QLabel("Z:");
    pointEditZ = new QLineEdit();
    pointEditZ->setText("0");
    pointEditZ->setMaximumWidth(150);
    pointBtn=new QPushButton("预置（点）");

    // 将标签和文本框添加到布局中
    firstLayout->addWidget(pointLabelX, 0, 0); // 第一行，第一列
    firstLayout->addWidget(pointEditX, 0, 1); // 第一行，第二列
    firstLayout->addWidget(pointLabelY, 1, 0); // 第二行，第一列
    firstLayout->addWidget(pointEditY, 1, 1); // 第二行，第二列
    firstLayout->addWidget(pointLabelZ, 2, 0); // 第三行，第一列
    firstLayout->addWidget(pointEditZ, 2, 1); // 第三行，第二列
    firstLayout->addWidget(pointBtn,4, 0, 1, 2);

    connect(pointBtn, &QPushButton::clicked, this, &PresetElemWidget::btnPointClicked);


    // 创建第二个tab的内容
    QWidget *secondTab = new QWidget();
    // 创建第2个tab的内容
    QGridLayout *secondLayout = new QGridLayout(secondTab);

    // 创建标签和文本框
    QLabel *lineLabelX1 = new QLabel("X:");
    lineEditX1 = new QLineEdit();
    lineEditX1->setText("0");
    lineEditX1->setMaximumWidth(150);
    QLabel *lineLabelY1 = new QLabel("Y:");
    lineEditY1 = new QLineEdit();
    lineEditY1->setText("0");
    lineEditY1->setMaximumWidth(150);
    QLabel *lineLabelZ1 = new QLabel("Z:");
    lineEditZ1 = new QLineEdit();
    lineEditZ1->setText("0");
    lineEditZ1->setMaximumWidth(150);

    QLabel *lineLabelX2 = new QLabel("X:");
    lineEditX2 = new QLineEdit();
    lineEditX2->setText("0");
    lineEditX2->setMaximumWidth(150);
    QLabel *lineLabelY2 = new QLabel("Y:");
    lineEditY2 = new QLineEdit();
    lineEditY2->setText("0");
    lineEditY2->setMaximumWidth(150);
    QLabel *lineLabelZ2 = new QLabel("Z:");
    lineEditZ2 = new QLineEdit();
    lineEditZ2->setText("0");
    lineEditZ2->setMaximumWidth(150);
    lineBtn=new QPushButton("预置（线）");

    // 将标签和文本框添加到布局中
    QGroupBox *groupBoxLeft = new QGroupBox("起点");
    QGridLayout *groupLeftLayout = new QGridLayout();
    groupLeftLayout->addWidget(lineLabelX1, 0, 0);
    groupLeftLayout->addWidget(lineEditX1, 0, 1);
    groupLeftLayout->addWidget(lineLabelY1, 1, 0);
    groupLeftLayout->addWidget(lineEditY1, 1, 1);
    groupLeftLayout->addWidget(lineLabelZ1, 2, 0);
    groupLeftLayout->addWidget(lineEditZ1, 2, 1);
    groupBoxLeft->setLayout(groupLeftLayout);

    QGroupBox *groupBoxRight = new QGroupBox("终点");
    QGridLayout *groupRightLayout = new QGridLayout();
    groupRightLayout->addWidget(lineLabelX2, 0, 0);
    groupRightLayout->addWidget(lineEditX2, 0, 1);
    groupRightLayout->addWidget(lineLabelY2, 1, 0);
    groupRightLayout->addWidget(lineEditY2, 1, 1);
    groupRightLayout->addWidget(lineLabelZ2, 2, 0);
    groupRightLayout->addWidget(lineEditZ2, 2, 1);
    groupBoxRight->setLayout(groupRightLayout);

    secondLayout->addWidget(groupBoxLeft, 0, 0);
    secondLayout->addWidget(groupBoxRight,0, 1);
    secondLayout->addWidget(lineBtn,3, 0, 1, 2);


    connect(lineBtn, &QPushButton::clicked, this, &PresetElemWidget::btnLineClicked);

    // //存储线坐标
    // dLine_X1_Type1=lineEditX1->text().toDouble();
    // dLine_Y1_Tpye1=lineEditY1->text().toDouble();
    // dLine_Z1_Tpye1=lineEditZ1->text().toDouble();
    // dLine_X2_Tpye1=lineEditX2->text().toDouble();
    // dLine_Y2_Tpye1=lineEditY2->text().toDouble();
    // dLine_Z2_Tpye1=lineEditZ2->text().toDouble();


    // 创建第三个tab的内容
    QWidget *thirdTab = new QWidget();

    // 创建网格布局
    QGroupBox *groupBox = new QGroupBox("请输入圆心的坐标:");
    QGridLayout *thirdLayout = new QGridLayout(thirdTab);

    // 创建标签和文本框
    QLabel *circleLabelX = new QLabel("X:");
    circleEditX = new QLineEdit();
    circleEditX->setText("0");
    circleEditX->setMaximumWidth(150);
    QLabel *circleLabelY = new QLabel("Y:");
    circleEditY = new QLineEdit();
    circleEditY->setText("0");
    circleEditY->setMaximumWidth(150);
    QLabel *circleLabelZ = new QLabel("Z:");
    circleEditZ = new QLineEdit();
    circleEditZ->setText("0");
    circleEditZ->setMaximumWidth(150);
    QLabel *circleLabelDiameter = new QLabel("直径:");
    circleEditD = new QLineEdit();
    circleEditD->setText("1");
    circleEditD->setMaximumWidth(150);

    circleBtn=new QPushButton("预置（圆）");

    // 将标签和文本框添加到布局中
    thirdLayout->addWidget(circleLabelX, 0, 0);
    thirdLayout->addWidget(circleEditX, 0, 1);
    thirdLayout->addWidget(circleLabelY, 1, 0);
    thirdLayout->addWidget(circleEditY, 1, 1);
    thirdLayout->addWidget(circleLabelZ, 2, 0);
    thirdLayout->addWidget(circleEditZ, 2, 1);
    thirdLayout->addWidget(circleLabelDiameter, 3, 0);
    thirdLayout->addWidget(circleEditD, 3, 1);
    thirdLayout->addWidget(circleBtn,4, 0, 1, 2);

    connect(circleBtn, &QPushButton::clicked, this, &PresetElemWidget::btnCircleClicked);


    // 创建第四个tab的内容
    QWidget *forthTab = new QWidget();
    QGridLayout *forthLayout = new QGridLayout(forthTab);
    // 将标签和文本框添加到布局中
    QGroupBox *groupBoxLeftUp = new QGroupBox("中心");
    QGridLayout *groupLeftUpLayout = new QGridLayout();
    QLabel *planeLabelX = new QLabel("X:");
    planeEditX = new QLineEdit();
    planeEditX->setText("0");
    QLabel *planeLabelY = new QLabel("Y:");
    planeEditY = new QLineEdit();
    planeEditY->setText("0");
    QLabel *planeLabelZ = new QLabel("Z:");
    planeEditZ = new QLineEdit();
    planeEditZ->setText("0");
    groupLeftUpLayout->addWidget(planeLabelX, 0, 0);
    groupLeftUpLayout->addWidget(planeEditX, 0, 1);
    groupLeftUpLayout->addWidget(planeLabelY, 1, 0);
    groupLeftUpLayout->addWidget(planeEditY, 1, 1);
    groupLeftUpLayout->addWidget(planeLabelZ, 2, 0);
    groupLeftUpLayout->addWidget(planeEditZ, 2, 1);
    groupBoxLeftUp->setLayout(groupLeftUpLayout);

    QGroupBox *groupBoxRightUp = new QGroupBox("尺寸");
    QGridLayout *groupRightUpLayout = new QGridLayout();
    QLabel *planeLabelL = new QLabel("Length:");
    planeEditL = new QLineEdit();
    planeEditL->setText("1");
    QLabel *planeLabelW = new QLabel("Width:");
    planeEditW = new QLineEdit();
    planeEditW->setText("1");
    groupRightUpLayout->addWidget(planeLabelL, 0, 0);
    groupRightUpLayout->addWidget(planeEditL, 0, 1);
    groupRightUpLayout->addWidget(planeLabelW, 1, 0);
    groupRightUpLayout->addWidget(planeEditW, 1, 1);
    groupBoxRightUp->setLayout(groupRightUpLayout);

    QGroupBox *groupBoxLeftDown = new QGroupBox("面法向");
    QGridLayout *groupLeftDownLayout = new QGridLayout();
    QLabel *planeLabelX2 = new QLabel("X:");
    planeNormalX = new QLineEdit();
    planeNormalX->setText("0");
    QLabel *planeLabelY2 = new QLabel("Y:");
    planeNormalY = new QLineEdit();
    planeNormalY->setText("0");
    QLabel *planeLabelZ2 = new QLabel("Z:");
    planeNormalZ = new QLineEdit();
    planeNormalZ->setText("1");
    groupLeftDownLayout->addWidget(planeLabelX2, 0, 0);
    groupLeftDownLayout->addWidget(planeNormalX, 0, 1);
    groupLeftDownLayout->addWidget(planeLabelY2, 1, 0);
    groupLeftDownLayout->addWidget(planeNormalY, 1, 1);
    groupLeftDownLayout->addWidget(planeLabelZ2, 2, 0);
    groupLeftDownLayout->addWidget(planeNormalZ, 2, 1);
    groupBoxLeftDown->setLayout(groupLeftDownLayout);

    QGroupBox *groupBoxRightDown = new QGroupBox("向量方向");
    QGridLayout *groupRightDownLayout = new QGridLayout();
    QLabel *planeLabelX3 = new QLabel("L:");
    planeDirectionX = new QLineEdit();
    planeDirectionX->setText("1");
    QLabel *planeLabelY3 = new QLabel("M:");
    planeDirectionY = new QLineEdit();
    planeDirectionY->setText("0");
    QLabel *planeLabelZ3 = new QLabel("N:");
    planeDirectionZ = new QLineEdit();
    planeDirectionZ->setText("0");
    groupRightDownLayout->addWidget(planeLabelX3, 0, 0);
    groupRightDownLayout->addWidget(planeDirectionX, 0, 1);
    groupRightDownLayout->addWidget(planeLabelY3, 1, 0);
    groupRightDownLayout->addWidget(planeDirectionY, 1, 1);
    groupRightDownLayout->addWidget(planeLabelZ3, 2, 0);
    groupRightDownLayout->addWidget(planeDirectionZ, 2, 1);
    groupBoxRightDown->setLayout(groupRightDownLayout);

    forthLayout->addWidget(groupBoxLeftUp, 0, 0);
    forthLayout->addWidget(groupBoxRightUp,0, 1);
    forthLayout->addWidget(groupBoxLeftDown, 1, 0);
    forthLayout->addWidget(groupBoxRightDown,1, 1);
    planeBtn=new QPushButton("预置（平面）");
    forthLayout->addWidget(planeBtn, 2, 0, 1, 2);
    connect(planeBtn, &QPushButton::clicked, this, &PresetElemWidget::btnplaneClicked);


    // 创建第五个tab的内容
    QWidget *fifthTab = new QWidget();
    QGridLayout *fifthLayout = new QGridLayout(fifthTab);
    QLabel *sphereLabelX = new QLabel("X:");
    sphereEditX = new QLineEdit();
    sphereEditX->setText("0");
    sphereEditX->setMaximumWidth(150);
    QLabel *sphereLabelY = new QLabel("Y:");
    sphereEditY = new QLineEdit();
    sphereEditY->setText("0");
    sphereEditY->setMaximumWidth(150);
    QLabel *sphereLabelZ = new QLabel("Z:");
    sphereEditZ = new QLineEdit();
    sphereEditZ->setText("0");
    sphereEditZ->setMaximumWidth(150);
    QLabel *sphereLabelD = new QLabel("Diametre:");
    sphereEditD = new QLineEdit();
    sphereEditD->setText("1");
    sphereEditD->setMaximumWidth(150);

    // 将标签和文本框添加到布局中
    fifthLayout->addWidget(sphereLabelX, 0, 0);
    fifthLayout->addWidget(sphereEditX, 0, 1);
    fifthLayout->addWidget(sphereLabelY, 1, 0);
    fifthLayout->addWidget(sphereEditY, 1, 1);
    fifthLayout->addWidget(sphereLabelZ, 2, 0);
    fifthLayout->addWidget(sphereEditZ, 2, 1);
    fifthLayout->addWidget(sphereLabelD, 3, 0);
    fifthLayout->addWidget(sphereEditD, 3, 1);
    sphereBtn=new QPushButton("预置（球）");
    fifthLayout->addWidget(sphereBtn,4, 0, 1, 2);

    connect(sphereBtn, &QPushButton::clicked, this, &PresetElemWidget::btnSphereClicked);


    // 创建第六个tab的内容
    QWidget *sixthTab = new QWidget();
    QGridLayout *sixthLayout = new QGridLayout(sixthTab);
    // 将标签和文本框添加到布局中
    QGroupBox *groupCylinderLeft = new QGroupBox("底面中心");
    QGridLayout *groupCylinderLeftLayout = new QGridLayout();
    QLabel *cylinderLabelX = new QLabel("X:");
    cylinderEditX = new QLineEdit();
    cylinderEditX->setText("0");
    QLabel *cylinderLabelY = new QLabel("Y:");
    cylinderEditY = new QLineEdit();
    cylinderEditY->setText("0");
    QLabel *cylinderLabelZ = new QLabel("Z:");
    cylinderEditZ = new QLineEdit();
    cylinderEditZ->setText("0");
    groupCylinderLeftLayout->addWidget(cylinderLabelX, 0, 0);
    groupCylinderLeftLayout->addWidget(cylinderEditX, 0, 1);
    groupCylinderLeftLayout->addWidget(cylinderLabelY, 1, 0);
    groupCylinderLeftLayout->addWidget(cylinderEditY, 1, 1);
    groupCylinderLeftLayout->addWidget(cylinderLabelZ, 2, 0);
    groupCylinderLeftLayout->addWidget(cylinderEditZ, 2, 1);
    groupCylinderLeft->setLayout(groupCylinderLeftLayout);

    QGroupBox *groupCylinderRight = new QGroupBox("轴向");
    QGridLayout *groupCylinderRightLayout = new QGridLayout();
    QLabel *cylinderLabelL = new QLabel("L:");
    cylinderNormalEditL = new QLineEdit();
    cylinderNormalEditL->setText("0");
    QLabel *cylinderLabelM = new QLabel("M:");
    cylinderNormalEditM = new QLineEdit();
    cylinderNormalEditM->setText("0");
    QLabel *cylinderLabelN = new QLabel("N:");
    cylinderNormalEditN = new QLineEdit();
    cylinderNormalEditN->setText("1");
    groupCylinderRightLayout->addWidget(cylinderLabelL, 0, 0);
    groupCylinderRightLayout->addWidget(cylinderNormalEditL, 0, 1);
    groupCylinderRightLayout->addWidget(cylinderLabelM, 1, 0);
    groupCylinderRightLayout->addWidget(cylinderNormalEditM, 1, 1);
    groupCylinderRightLayout->addWidget(cylinderLabelN, 2, 0);
    groupCylinderRightLayout->addWidget(cylinderNormalEditN, 2, 1);
    groupCylinderRight->setLayout(groupCylinderRightLayout);

    QLabel *cylinderLabelD = new QLabel("直径:");
    QLabel *cylinderLabelH = new QLabel("高度:");
    cylinderEditD = new QLineEdit();
    cylinderEditD->setText("1");
    cylinderEditH = new QLineEdit();
    cylinderEditH->setText("1");
    sixthLayout->addWidget(groupCylinderLeft, 0, 0);
    sixthLayout->addWidget(groupCylinderRight, 0, 1);
    sixthLayout->addWidget(cylinderLabelD, 1, 0);
    sixthLayout->addWidget(cylinderEditD, 1, 1);
    sixthLayout->addWidget(cylinderLabelH, 2, 0);
    sixthLayout->addWidget(cylinderEditH, 2, 1);

    cylinderBtn=new QPushButton("预置（圆柱体）");
    sixthLayout->addWidget(cylinderBtn,3, 0, 1, 2);

    connect(cylinderBtn, &QPushButton::clicked, this, &PresetElemWidget::btnCylinderClicked);


    // 创建第七个tab的内容
    QWidget *seventhTab = new QWidget();
    QGridLayout *seventhLayout = new QGridLayout(seventhTab);
    // 将标签和文本框添加到布局中
    QGroupBox *groupConeLeft = new QGroupBox("底面中心");
    QGridLayout *groupConeLeftLayout = new QGridLayout();
    QLabel *coneLabelX = new QLabel("X:");
    coneEditX = new QLineEdit();
    coneEditX->setText("0");
    QLabel *coneLabelY = new QLabel("Y:");
    coneEditY = new QLineEdit();
    coneEditY->setText("0");
    QLabel *coneLabelZ = new QLabel("Z:");
    coneEditZ = new QLineEdit();
    coneEditZ->setText("0");
    groupConeLeftLayout->addWidget(coneLabelX, 0, 0);
    groupConeLeftLayout->addWidget(coneEditX, 0, 1);
    groupConeLeftLayout->addWidget(coneLabelY, 1, 0);
    groupConeLeftLayout->addWidget(coneEditY, 1, 1);
    groupConeLeftLayout->addWidget(coneLabelZ, 2, 0);
    groupConeLeftLayout->addWidget(coneEditZ, 2, 1);
    groupConeLeft->setLayout(groupConeLeftLayout);

    QGroupBox *groupConeRight = new QGroupBox("轴向");
    QGridLayout *groupConeRightLayout = new QGridLayout();
    QLabel *coneLabelL = new QLabel("L:");
    coneNormalEditL = new QLineEdit();
    coneNormalEditL->setText("0");
    QLabel *coneLabelM = new QLabel("M:");
    coneNormalEditM = new QLineEdit();
    coneNormalEditM->setText("0");
    QLabel *coneLabelN = new QLabel("N:");
    coneNormalEditN = new QLineEdit();
    coneNormalEditN->setText("1");
    groupConeRightLayout->addWidget(coneLabelL, 0, 0);
    groupConeRightLayout->addWidget(coneNormalEditL, 0, 1);
    groupConeRightLayout->addWidget(coneLabelM, 1, 0);
    groupConeRightLayout->addWidget(coneNormalEditM, 1, 1);
    groupConeRightLayout->addWidget(coneLabelN, 2, 0);
    groupConeRightLayout->addWidget(coneNormalEditN, 2, 1);
    groupConeRight->setLayout(groupConeRightLayout);

    QLabel *coneLabelpartH = new QLabel("圆台高度:");
    QLabel *coneLabelFullH = new QLabel("总高度:");
    coneEditpartH = new QLineEdit();
    coneEditpartH->setText("1");
    coneEditFullH = new QLineEdit();
    coneEditFullH->setText("1");
    QLabel *coneLabelAngle = new QLabel("半角:");
    coneEditAngle = new QLineEdit();
    coneEditAngle->setText("15.000");
    seventhLayout->addWidget(groupConeLeft, 0, 0);
    seventhLayout->addWidget(groupConeRight, 0, 1);
    seventhLayout->addWidget(coneLabelpartH, 1, 0);
    seventhLayout->addWidget(coneEditpartH, 1, 1);
    seventhLayout->addWidget(coneLabelFullH, 2, 0);
    seventhLayout->addWidget(coneEditFullH, 2, 1);
    seventhLayout->addWidget(coneLabelAngle, 3, 0);
    seventhLayout->addWidget(coneEditAngle, 3, 1);

    coneBtn=new QPushButton("预置（圆锥）");
    seventhLayout->addWidget(coneBtn,4, 0, 1, 2);

    connect(coneBtn, &QPushButton::clicked, this, &PresetElemWidget::btnConeClicked);


    // 将tab页面添加到tab widget
    tabWidget->addTab(firstTab, "点");
    tabWidget->addTab(secondTab, "线");
    tabWidget->addTab(thirdTab, "圆");
    tabWidget->addTab(forthTab, "平面");
    tabWidget->addTab(fifthTab, "球");
    tabWidget->addTab(sixthTab, "圆柱");
    tabWidget->addTab(seventhTab, "圆锥");

    // 显示tab widget
    tabWidget->setAttribute(Qt::WA_DeleteOnClose); // 关闭窗口时释放资源
    tabWidget->resize(480, 320);  // 设置宽度为400，高度为300
    tabWidget->setCurrentIndex(2);
    tabWidget->show();

}

//槽函数的定义
void PresetElemWidget::btnCircleClicked()
{
    CPosition mypt; //坐标
    bool ok;
    mypt.x = circleEditX->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆-坐标X需要是双精度浮点类型数。");
        return;
    }
    mypt.y = circleEditY->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆-坐标Y需要是双精度浮点类型数。");
        return;
    }
    mypt.z = circleEditZ->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆-坐标Z需要是双精度浮点类型数。");
        return;
    }
    double d = circleEditD->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "直径D需要是双精度浮点类型数。");
        return;
    }
    //m_pMainWin->OnPresetCircle(mypt, d);
    // qDebug()<<"clicked";

}

void PresetElemWidget::btnPointClicked()
{
    qDebug()<<"before clicked";
    CPosition mypt;
    bool ok;
    mypt.x = pointEditX->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "点-坐标X需要是双精度浮点类型数。");
        return;
    }
    mypt.y = pointEditY->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "点-坐标Y需要是双精度浮点类型数。");
        return;
    }
    mypt.z = pointEditZ->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "点-坐标Z需要是双精度浮点类型数。");
        return;
    }
    qDebug()<<"clicked1";
    //m_pMainWin->OnPresetPoint(mypt);
    qDebug()<<"clicked2";
}

void PresetElemWidget::btnLineClicked()
{
    qDebug()<<"before clicked";
    CPosition ptBegin;
    bool ok;
    ptBegin.x = lineEditX1->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "线-起点坐标X需要是双精度浮点类型数。");
        return;
    }
    ptBegin.y = lineEditY1->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "线-起点坐标Y需要是双精度浮点类型数。");
        return;
    }
    ptBegin.z = lineEditZ1->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "线-起点坐标Z需要是双精度浮点类型数。");
        return;
    }
    qDebug()<<ptBegin.x<<ptBegin.y<<ptBegin.z;
    CPosition ptEnd;
    ptEnd.x = lineEditX2->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "线-终点坐标X需要是双精度浮点类型数。");
        return;
    }
    ptEnd.y = lineEditY2->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "线-终点坐标Y需要是双精度浮点类型数。");
        return;
    }
    ptEnd.z = lineEditZ2->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "线-终点坐标Z需要是双精度浮点类型数。");
        return;
    }
    qDebug()<<ptEnd.x<<ptEnd.y<<ptEnd.z;
    qDebug()<<"clicked1";
    //m_pMainWin->OnPresetLine(ptBegin, ptEnd);
    qDebug()<<"clicked2##";

}

void PresetElemWidget::btnplaneClicked()
{
    qDebug()<<"before clicked";
    CPosition center;
    bool ok;
    center.x = planeEditX->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "平面-中心坐标X需要是双精度浮点类型数。");
        return;
    }
    center.y = planeEditY->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "平面-中心坐标Y需要是双精度浮点类型数。");
        return;
    }
    center.z = planeEditZ->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "平面-中心坐标Z需要是双精度浮点类型数。");
        return;
    }

    QVector4D vecNormal;
    vecNormal.setX(planeNormalX->text().toDouble(&ok));
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "平面-法线坐标X需要是双精度浮点类型数。");
        return;
    }
    vecNormal.setY(planeNormalY->text().toDouble(&ok));
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "平面-法线坐标Y需要是双精度浮点类型数。");
        return;
    }
    vecNormal.setZ(planeNormalZ->text().toDouble(&ok));
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "平面-法线坐标Z需要是双精度浮点类型数。");
        return;
    }

    double length = 0;
    double width = 0;
    length = planeEditL->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "平面-长度L需要是双精度浮点类型数。");
        return;
    }
    width = planeEditW->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "平面-宽度L需要是双精度浮点类型数。");
        return;
    }

    QVector4D vecDirection;
    vecDirection.setX(planeDirectionX->text().toDouble(&ok));
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "平面-长边方向坐标X需要是双精度浮点类型数。");
        return;
    }
    vecDirection.setY(planeDirectionY->text().toDouble(&ok));
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "平面-长边方向坐标Y需要是双精度浮点类型数。");
        return;
    }
    vecDirection.setZ(planeDirectionZ->text().toDouble(&ok));
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "平面-长边方向坐标Z需要是双精度浮点类型数。");
        return;
    }
    //m_pMainWin->OnPresetPlane(center, vecNormal, vecDirection,length, width);

}

void PresetElemWidget::btnSphereClicked()
{
    CPosition center;
    bool ok;
    center.x = sphereEditX->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "球体-中心坐标X需要是双精度浮点类型数。");
        return;
    }
    center.y = sphereEditY->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "球体-中心坐标Y需要是双精度浮点类型数。");
        return;
    }
    center.z = sphereEditZ->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "球体-中心坐标Z需要是双精度浮点类型数。");
        return;
    }

    double diametre = 0;
    diametre = sphereEditD->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "球体-直径D需要是双精度浮点类型数。");
        return;
    }
    //m_pMainWin->OnPresetSphere(center, diametre);
}

void PresetElemWidget::btnCylinderClicked()
{
    qDebug()<<"try to add a Cyinder";
    CPosition center;
    bool ok;
    center.x = cylinderEditX->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆柱体-底面中心坐标X需要是双精度浮点类型数。");
        return;
    }
    center.y = cylinderEditY->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆柱体-底面中心坐标Y需要是双精度浮点类型数。");
        return;
    }
    center.z = cylinderEditZ->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆柱体-底面中心坐标Z需要是双精度浮点类型数。");
        return;
    }

    CPosition vec;
    vec.x = cylinderNormalEditL->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆柱体-轴向法线L需要是双精度浮点类型数。");
        return;
    }
    vec.y = cylinderNormalEditM->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆柱体-轴向法线M需要是双精度浮点类型数。");
        return;
    }
    vec.z = cylinderNormalEditN->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆柱体-轴向法线N需要是双精度浮点类型数。");
        return;
    }

    double height=0;
    height = cylinderEditH->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆柱体-高度Height需要是双精度浮点类型数。");
        return;
    }
    double diametre=0;
    diametre = cylinderEditD->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆柱体-直径需要是双精度浮点类型数。");
        return;
    }
    QVector4D myvec(vec.x, vec.y, vec.z, 1);

    qDebug()<<"try to add a Cyinder2";
    //m_pMainWin->OnPresetCylinder(center, myvec, height, diametre);



}

void PresetElemWidget::btnConeClicked()
{

    CPosition center;
    bool ok;
    center.x = coneEditX->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆锥-底面中心坐标X需要是双精度浮点类型数。");
        return;
    }
    center.y = coneEditY->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆锥-底面中心坐标Y需要是双精度浮点类型数。");
        return;
    }
    center.z = coneEditZ->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆锥-底面中心坐标Z需要是双精度浮点类型数。");
        return;
    }

    CPosition vec;
    vec.x = coneNormalEditL->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆锥-轴向法线L需要是双精度浮点类型数。");
        return;
    }
    vec.y = coneNormalEditM->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆锥-轴向法线M需要是双精度浮点类型数。");
        return;
    }
    vec.z = coneNormalEditN->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆锥-轴向法线N需要是双精度浮点类型数。");
        return;
    }

    double partHeight=0;
    partHeight = coneEditpartH->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆锥-高度Height需要是双精度浮点类型数。");
        return;
    }
    double fullHeight=0;
    fullHeight = coneEditFullH->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆锥-总高度需要是双精度浮点类型数。");
        return;
    }

    double angle=0;
    angle = coneEditAngle->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆锥-角度需要是双精度浮点类型数。");
        return;
    }
    QVector4D myvec(vec.x, vec.y, vec.z, 1);

    qDebug()<<"try to add a Cyinder2";
    //m_pMainWin->OnPresetCone(center, myvec, partHeight, fullHeight, angle);

}


#ifndef PRESETELEMWIDGET_H
#define PRESETELEMWIDGET_H

#include <QWidget>
#include <QTabWidget>
#include<QLineEdit>
#include<QPushButton>

class MainWindow;

class PresetElemWidget : public QWidget
{
    Q_OBJECT
public:
    explicit PresetElemWidget(QWidget *parent = nullptr);

    MainWindow* m_pMainWin;

    // 创建tab widget
    QTabWidget* tabWidget;

    //预置点
    QLineEdit* pointEditX;
    QLineEdit* pointEditY;
    QLineEdit* pointEditZ;
    QPushButton* pointBtn;

    //预置线
    QLineEdit *lineEditX1;
    QLineEdit *lineEditY1;
    QLineEdit *lineEditZ1;
    QLineEdit *lineEditX2;
    QLineEdit *lineEditY2;
    QLineEdit *lineEditZ2;
    QLineEdit *lineEditX3;
    QLineEdit *lineEditY3;
    QLineEdit *lineEditZ3;
    QLineEdit *lineEditAngle;
    QLineEdit *lineEditLength;
    QPushButton* lineBtn;
    //内部存储线坐标
    double dLine_X1_Type1;
    double dLine_Y1_Tpye1;
    double dLine_Z1_Tpye1;
    double dLine_X2_Tpye1;
    double dLine_Y2_Tpye1;
    double dLine_Z2_Tpye1;

    //预置圆
    QLineEdit* circleEditX;
    QLineEdit* circleEditY;
    QLineEdit* circleEditZ;
    QLineEdit* circleEditD; //直径
    QPushButton* circleBtn;

    //预置平面
    QLineEdit *planeEditX;
    QLineEdit *planeEditY;
    QLineEdit *planeEditZ;
    QLineEdit *planeEditL;
    QLineEdit *planeEditW;
    QLineEdit *planeNormalX;
    QLineEdit *planeNormalY;
    QLineEdit *planeNormalZ;
    QLineEdit *planeDirectionX;
    QLineEdit *planeDirectionY;
    QLineEdit *planeDirectionZ;
    QPushButton *planeBtn;

    //预置球体
    QLineEdit *sphereEditX;
    QLineEdit *sphereEditY;
    QLineEdit *sphereEditZ;
    QLineEdit *sphereEditD;
    QPushButton *sphereBtn;

    //预置圆柱体
    QLineEdit *cylinderEditX;
    QLineEdit *cylinderEditY;
    QLineEdit *cylinderEditZ;
    QLineEdit *cylinderEditD;
    QLineEdit *cylinderEditH;
    QLineEdit *cylinderNormalEditL;
    QLineEdit *cylinderNormalEditM;
    QLineEdit *cylinderNormalEditN;
    QPushButton *cylinderBtn;

    //预置圆锥体
    QLineEdit *coneEditX;
    QLineEdit *coneEditY;
    QLineEdit *coneEditZ;
    QLineEdit *coneEditpartH;
    QLineEdit *coneEditFullH;
    QLineEdit *coneNormalEditL;
    QLineEdit *coneNormalEditM;
    QLineEdit *coneNormalEditN;
    QLineEdit *coneEditAngle;
    QPushButton *coneBtn;

    //预置长方体
    QLineEdit *boxEditX;
    QLineEdit *boxEditY;
    QLineEdit *boxEditZ;
    QLineEdit *boxEditLength;
    QLineEdit *boxEditWidth;
    QLineEdit *boxEditHeight;
    QLineEdit *boxEditNormalX;
    QLineEdit *boxEditNormalY;
    QLineEdit *boxEditNormalZ;
    QPushButton *boxBtn;

    //槽函数
    void btnPointClicked();
    void btnLineClicked();
    void btnCircleClicked();
    void btnplaneClicked();
    void btnSphereClicked();
    void btnCylinderClicked();
    void btnConeClicked();
    void btnBoxClicked();


signals:
};

#endif // PRESETELEMWIDGET_H

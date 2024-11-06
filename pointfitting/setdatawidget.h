#ifndef SETDATAWIDGET_H
#define SETDATAWIDGET_H

#include <QWidget>
#include <QDialog>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

class MainWindow;
class setDataWidget : public QWidget
{
    Q_OBJECT
public:
    explicit setDataWidget(QWidget *parent = nullptr);

    void setPlaneData();//设置拟合平面的领域和距离阈值的对话框
    void PlaneBtnClick();

private:
    MainWindow *m_pMainWin;

    //拟合平面对话框
    QDialog *p_dialog;
    QGridLayout *p_layout;
    QLabel *p_lab1;
    QLabel *p_lab2;
    QLineEdit *p_rad;
    QLineEdit *p_dis;
    QPushButton *p_btn;

signals:
};

#endif // SETDATAWIDGET_H

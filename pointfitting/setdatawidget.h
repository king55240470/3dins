#ifndef SETDATAWIDGET_H
#define SETDATAWIDGET_H

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

#include <QWidget>
#include <QDialog>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

class MainWindow;
class FittingPlane;
class setDataWidget : public QWidget
{
    Q_OBJECT
public:
    explicit setDataWidget(QWidget *parent = nullptr);

    void setPlaneData(pcl::PointXYZRGB,pcl::PointCloud<pcl::PointXYZRGB>::Ptr);//设置拟合平面的领域和距离阈值的对话框
    void PlaneBtnClick();

private:
    MainWindow *m_pMainWin;

    //拟合平面对话框
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fittingPlane;
    QDialog *p_dialog;
    QGridLayout *p_layout;
    QLabel *p_lab1;
    QLabel *p_lab2;
    QLineEdit *p_rad;
    QLineEdit *p_dis;
    QPushButton *p_btn;
    pcl::PointXYZRGB p_point;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloudptr;

signals:
};

#endif // SETDATAWIDGET_H

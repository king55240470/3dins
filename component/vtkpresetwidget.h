#ifndef VTKPRESETWIDGET_H
#define VTKPRESETWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include "manager/centitymgr.h"

class VtkPresetWidget : public QWidget
{
    Q_OBJECT
public:
    explicit VtkPresetWidget(QWidget *parent = nullptr);

    // 获取m_renWin
    vtkSmartPointer<vtkRenderWindow> getRenderWindow();

    // 将draw返回的actor添加到渲染器中
    void addActor(vtkSmartPointer<vtkActor>& actor);

private:
    QLabel* label;
    CEntityMgr* m_entityMgr; // 创建一个管理器对象用于操作预置的元素

    // 为m_entityList创建Qvector用来封装执行器，每个元素都有一个自己的actor，用于显示图形
    QVector<vtkSmartPointer<vtkActor>> m_vtkActors;

    // 为所有的actor创建渲染器、渲染窗口和交互器
    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkRenderWindow> m_renWin;
    vtkSmartPointer<vtkRenderWindowInteractor> m_interactor;

signals:
};

#endif // VTKPRESETWIDGET_H

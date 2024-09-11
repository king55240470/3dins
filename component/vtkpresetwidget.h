#ifndef VTKPRESETWIDGET_H
#define VTKPRESETWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include "manager/centitymgr.h"
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

class VtkPresetWidget : public QWidget
{
    Q_OBJECT
public:
    explicit VtkPresetWidget(QWidget *parent = nullptr);

    // 获取m_renWin
    vtkSmartPointer<vtkRenderWindow> getRenderWindow();

    // 将draw返回的actor添加到渲染器中
    static void addActor(vtkSmartPointer<vtkActor>& actor);

private:
    QLabel* label;
    CEntityMgr* m_entityMgr; // 创建一个管理器对象用于操作预置的元素

    // 为所有的actor创建渲染器、渲染窗口和交互器
    static vtkSmartPointer<vtkRenderer> m_renderer;
    static vtkSmartPointer<vtkRenderWindow> m_renWin;
    static vtkSmartPointer<vtkRenderWindowInteractor> m_interactor;

signals:
};

#endif // VTKPRESETWIDGET_H

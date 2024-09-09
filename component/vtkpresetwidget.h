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
    vtkSmartPointer<vtkRenderWindow> GetRenderWindow();

    // 由centity对象生成一个actor
    vtkSmartPointer<vtkActor> CreateActorFromEntity(CEntity* entity);

    // 为m_entityList的新元素分配actor并存入m_vtkActors
    void addActor(vtkSmartPointer<vtkActor> new_actor);

private:
    QLabel* label;
    CEntityMgr* m_entityMgr;

    // 为m_entityList创建Qvector用来封装执行器，每个元素都有一个自己的actor，用于显示图形
    QVector<vtkSmartPointer<vtkActor>> m_vtkActors;

    // 为所有的actor创建渲染器、渲染窗口和交互器
    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkRenderWindow> m_renWin;
    vtkSmartPointer<vtkRenderWindowInteractor> m_interactor;

signals:
};

#endif // VTKPRESETWIDGET_H

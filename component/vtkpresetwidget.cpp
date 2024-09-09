#include "vtkpresetwidget.h"

#include <QVTKOpenGLWidget.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

VtkPresetWidget::VtkPresetWidget(QWidget *parent)
    : QWidget{parent}
{
    label=new QLabel("预置的图形显示",this);

    // 为管理器分配内存
    m_entityMgr = new CEntityMgr();
    // 渲染器、渲染窗口和交互器分配内存
    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    m_renWin = vtkSmartPointer<vtkRenderWindow>::New();
    m_interactor= vtkSmartPointer<vtkRenderWindowInteractor>::New();

    // 创建球体源
    vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetCenter(0, 2, 0);
    sphere->SetRadius(1);
    sphere->SetEndTheta(360);
    sphere->SetThetaResolution(200); // 分辨率
    // 创建映射器
    auto mapper_2 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_2->SetInputConnection(sphere->GetOutputPort());
    // 创建执行器
    auto actor_2 = vtkSmartPointer<vtkActor>::New();
    actor_2->SetMapper(mapper_2);
    actor_2->GetProperty()->SetColor(0.7, 0.3, 0.3);
    actor_2->GetProperty()->SetRepresentationToWireframe(); // 设置成线框模式
    // 添加到渲染器
    m_renderer->AddActor(actor_2);
    // 将渲染器添加到渲染窗口
    m_renWin->AddRenderer(m_renderer);

    // 设置 VTK 渲染窗口到 QWidget
    QVTKOpenGLNativeWidget* vtkWidget = new QVTKOpenGLNativeWidget(this);
    vtkWidget->setRenderWindow(m_renWin);
    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->addWidget(vtkWidget);
    setLayout(layout);

}

// 根据entity的类型分配mapper和actor
vtkSmartPointer<vtkRenderWindow> VtkPresetWidget::GetRenderWindow(){

    return nullptr;
}

// 传入centity子类对象，生成一个actor
vtkSmartPointer<vtkActor> VtkPresetWidget::CreateActorFromEntity(CEntity* entity){
    entity->GetEntityType();
    return nullptr;
}

void VtkPresetWidget::addActor(vtkSmartPointer<vtkActor> new_actor){
    auto pEntity = m_entityMgr->getLatestEntity();

    // 为m_entityList新加的元素分配一个actor
    vtkSmartPointer<vtkActor> actor = CreateActorFromEntity(pEntity);
    if (actor) {
        m_vtkActors.push_back(actor);
        m_renderer->AddActor(actor);
    }
}

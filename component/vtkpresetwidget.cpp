#include "vtkpresetwidget.h"
#include "manager/centitymgr.h"

#include <QVTKOpenGLWidget.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

// 定义并初始化渲染器、渲染窗口和交互器
vtkSmartPointer<vtkRenderer> VtkPresetWidget::m_renderer = vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkRenderWindow> VtkPresetWidget::m_renWin = vtkSmartPointer<vtkRenderWindow>::New();
vtkSmartPointer<vtkRenderWindowInteractor> VtkPresetWidget::m_interactor= vtkSmartPointer<vtkRenderWindowInteractor>::New();

VtkPresetWidget::VtkPresetWidget(QWidget *parent)
    : QWidget{parent}
{
    label=new QLabel("预置的图形显示",this);
    // 创建管理器
    m_entityMgr = new CEntityMgr();

    // 为交互器设置窗口
    m_interactor->SetRenderWindow(m_renWin);

    // 创建球体源
    vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetCenter(0, 2, 0);
    sphere->SetRadius(1);
    sphere->SetEndTheta(360);
    sphere->SetThetaResolution(50); // 分辨率
    // 创建映射器
    auto mapper_2 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_2->SetInputConnection(sphere->GetOutputPort());
    // 创建执行器
    auto actor_2 = vtkSmartPointer<vtkActor>::New();
    actor_2->SetMapper(mapper_2);
    actor_2->GetProperty()->SetColor(0.7, 0.3, 0.3);
    // actor_2->GetProperty()->SetRepresentationToWireframe(); // 设置成线框模式

    m_renderer->SetBackground(0, 0, 0);
    m_renderer->AddActor(actor_2); // 添加到渲染器
    m_renWin->AddRenderer(m_renderer);  // 将渲染器添加到渲染窗口

    // 设置 VTK 渲染窗口到 QWidget
    QVTKOpenGLNativeWidget* vtkWidget = new QVTKOpenGLNativeWidget(this);
    vtkWidget->setRenderWindow(m_renWin);
    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->addWidget(vtkWidget);
    setLayout(layout);

    m_renWin->Render(); // 开始渲染
}

vtkSmartPointer<vtkRenderWindow> VtkPresetWidget::getRenderWindow(){
    return m_renWin;
}

void VtkPresetWidget::addActor(vtkSmartPointer<vtkActor>& actor){
    m_renderer->AddActor(actor);
}




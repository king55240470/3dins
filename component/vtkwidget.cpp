#include "vtkwidget.h"

// 渲染窗口大小
#define WIDTH 1000
#define HEIGHT 800

VtkWidget::VtkWidget(QWidget *parent)
    : QWidget(parent)
{
    m_pMainWin = (MainWindow*) parent;

    // 初始化渲染器和交互器
    renWin = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renWin->AddRenderer(renderer);  // 将渲染器添加到渲染窗口
    // interactor = vtkSmartPointer<vtkGenericRenderWindowInteractor>::New();
    // interactor->SetRenderWindow(renWin);

    // 设置 VTK 渲染窗口到 QWidget
    QHBoxLayout *mainlayout = new QHBoxLayout;
    setUpVtk(mainlayout); // 配置vtk窗口
    this->setLayout(mainlayout);

}

vtkSmartPointer<vtkRenderWindow> VtkWidget::getRenderWindow(){
    return renWin;
}

vtkSmartPointer<vtkRenderer>& VtkWidget::getRenderer(){
    // return m_renderer;
    return renderer;
}

void VtkWidget::addActor(vtkSmartPointer<vtkActor> actor){
    renderer->AddActor(actor);
}

void VtkWidget::UpdateInfo(){
    reDraw();
}

void VtkWidget::reDraw(){
    // 清除渲染器中的所有 actor
    auto* actorCollection = getRenderer()->GetActors();

    // 创建一个迭代器用于遍历actor集合
    vtkCollectionSimpleIterator it;

    // 初始化迭代器，准备遍历actor集合
    actorCollection->InitTraversal(it);

    vtkSmartPointer<vtkActor> actor;
    while ((actor = actorCollection->GetNextActor(it)) != nullptr)
    {
        renderer->RemoveActor(actor);
    }

    // 遍历m_entityList重新绘制
    for(auto& entity : m_pMainWin->m_EntityListMgr->getEntityList()){
        addActor(entity->draw());
    }

    getRenderWindow()->Render(); // 刷新渲染窗口
}

// 创建坐标器
void VtkWidget::createAxes()
{
    // 创建坐标器
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    // 设置 X Y Z 轴标题颜色为黑色
    axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);

    // 设置字体大小
    axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
    axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
    axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);

    axes->SetTotalLength(0.3, 0.3, 0.3); // 设置轴的长度
    axes->SetConeRadius(0.08); // 设置轴锥体的半径
    axes->SetCylinderRadius(0.1); // 设置轴圆柱体的半径
    axes->SetSphereRadius(0.05); // 设置轴末端的球体半径
    axes->SetPosition(0, 0, 0);
    renderer->AddActor(axes); // 将坐标器添加到渲染器

    // orientationWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    // // 设置 X Y Z 轴标题颜色为黑色
    // axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    // axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    // axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);

    // // 将坐标轴演员添加到orientationWidget
    // orientationWidget->SetOrientationMarker(axes);
    // // 将orientationWidget与交互器关联
    // orientationWidget->SetInteractor(interactor);
    // // 设置视口
    // orientationWidget->SetViewport(0.0, 0.0, 0.2, 0.2);

    // // orientationWidget->SetEnabled(1);
    // orientationWidget->InteractiveOff();
}

void VtkWidget::setUpVtk(QHBoxLayout *layout){
    createAxes();

    //创建视角相机
    vtkCamera* camera = renderer->GetActiveCamera();
    if (camera) {
        // 设置相机的初始位置和焦点
        camera->SetPosition(0, 0, 1);
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 1, 0);

        // 根据需要调整视野角度
        camera->SetViewAngle(60);

        // 根据场景的具体大小和需要调整裁剪范围
        camera->SetClippingRange(0.1, 1000);

        // 根据需要调整视图的缩放
        camera->Zoom(0.45);
    }

    // 设置渲染器颜色为白
    // renderer->SetBackground(0.1, 0.2, 0.4);
    renderer->SetBackground(1, 1, 1);

    QVTKOpenGLNativeWidget* vtkWidget = new QVTKOpenGLNativeWidget(this);
    vtkWidget->setRenderWindow(renWin);
    layout->addWidget(vtkWidget);
    renWin->Render();
}

VtkWidget::~VtkWidget() {}


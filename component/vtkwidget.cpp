#include "vtkwidget.h"
#include <vtkInteractorStyle.h>

// 渲染窗口大小
#define WIDTH 1000
#define HEIGHT 800

VtkWidget::VtkWidget(QWidget *parent)
    : QWidget(parent)
{
    m_pMainWin = (MainWindow*) parent;

    // 设置 VTK 渲染窗口到 QWidget
    QVBoxLayout *mainlayout = new QVBoxLayout;
    setUpVtk(mainlayout); // 配置vtk窗口
    this->setLayout(mainlayout);

}

void VtkWidget::setUpVtk(QVBoxLayout *layout){
    // 初始化渲染器和交互器
    renWin = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renWin->AddRenderer(renderer);  // 将渲染器添加到渲染窗口
    // interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    // interactor->SetRenderWindow(renWin);

    // 创建坐标器
    createAxes();
    // 创建初始视角相机
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
        camera->Zoom(0.5);
    }

    // 设置渲染器颜色为白
    renderer->SetBackground(1, 1, 1);

    vtkWidget = new QVTKOpenGLNativeWidget(this);
    vtkWidget->setRenderWindow(renWin);
    layout->addWidget(vtkWidget);

    // // 创建交互器样式
    // auto customStyle = vtkSmartPointer<vtkInteractorStyle>::New();
    // customStyle->SetInteractor(interactor);
    // customStyle->SetCurrentRenderer(renderer.GetPointer()); // 设置当前的渲染器
    // // 设置VTK窗口的交互样式
    // renWin->GetInteractor()->SetInteractorStyle(customStyle);

    renWin->Render();
}

vtkSmartPointer<vtkRenderWindow> VtkWidget::getRenderWindow(){
    return renWin;
}

vtkSmartPointer<vtkRenderer>& VtkWidget::getRenderer(){
    // return m_renderer;
    return renderer;
}

void VtkWidget::UpdateInfo(){
    reDraw();
}

void VtkWidget::reDraw(){
    // 获取渲染器中的所有 actor
    auto* actorCollection = getRenderer()->GetActors();

    // 创建一个迭代器用于遍历actor集合
    vtkCollectionSimpleIterator it;

    // 初始化迭代器，准备遍历actor集合
    actorCollection->InitTraversal(it);

    // vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkProp3D> prop;
    // 遍历并移除 vtkProp3D 对象（包括 vtkActor 和 vtkAxesActor）
    while ((prop = actorCollection->GetNextActor(it)) != nullptr)
    {
        renderer->RemoveActor(prop);
    }

    QVector<bool> list = m_pMainWin->m_EntityListMgr->getMarkList();//获取标记是否隐藏元素的list
    // 存储返回的引用对象，用于操作两个list
    auto entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    auto objectlist = m_pMainWin->m_ObjectListMgr->getObjectList();

    // 遍历entitylist绘制图形并加入渲染器
    for(auto i = 0;i < entitylist.size();i++){
        if(!list[i]){
            getRenderer()->AddActor(entitylist[i]->draw());
        }
    }
    // 遍历objectlist绘制坐标系并加入渲染器
    for(auto object:objectlist){
        getRenderer()->AddActor(object->draw());
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
    axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(5);
    axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(5);
    axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(5);

    axes->SetTotalLength(0.4, 0.4, 0.4); // 设置轴的长度
    axes->SetConeRadius(0.1); // 设置轴锥体的半径
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
    // orientationWidget->SetInteractor(renWin->GetInteractor());
    // // 设置视口
    // orientationWidget->SetViewport(0.0, 0.0, 0.2, 0.2);

    // orientationWidget->SetEnabled(true);
    // orientationWidget->InteractiveOff();
}

// 切换相机视角1
void VtkWidget::onTopView() {
    vtkCamera *camera = renderer->GetActiveCamera();
    if (camera) {
        camera->SetPosition(0, 0, 1);  // 重置相机位置为俯视
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 1, 0);

        camera->OrthogonalizeViewUp(); // 确保与SetViewUp方向正交

        // 重新设置相机并渲染
        renderer->ResetCamera();
        renWin->Render();
    }
}

// 切换相机视角2
void VtkWidget::onRightView(){
    vtkCamera *camera = renderer->GetActiveCamera();
    if (camera) {
        camera->SetPosition(1, 0, 0);  // 重置相机位置为右侧
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 1, 0);

        camera->OrthogonalizeViewUp(); // 确保与SetViewUp方向正交

        // 重新设置相机并渲染
        renderer->ResetCamera();
        renWin->Render();
    }
}

// 切换相机视角3
void VtkWidget::onFrontView(){
    vtkCamera *camera = renderer->GetActiveCamera();
    if (camera) {
        camera->SetPosition(0, -1, 0);  // 重置相机位置为正视
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 0, 1);

        camera->OrthogonalizeViewUp(); // 确保与SetViewUp方向正交

        // 重新设置相机并渲染
        renderer->ResetCamera();
        renWin->Render();
    }
}

// 切换相机视角4，立体视角可以在前三个的基础上旋转
void VtkWidget::ononIsometricView(){
    vtkCamera *camera = renderer->GetActiveCamera();
    if (camera) {
        camera->SetPosition(0, 0, 0); // 重置相机位置
        camera->SetViewUp(0, 1, 0);    // 重置视角向上方向

        camera->Azimuth(30);
        camera->Elevation(30);
        camera->OrthogonalizeViewUp();

        // 重新设置相机并渲染
        renderer->ResetCamera();
        renWin->Render();
    }
}


VtkWidget::~VtkWidget() {}


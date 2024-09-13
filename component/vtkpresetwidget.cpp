#include "vtkpresetwidget.h"
// #include "manager/centitymgr.h"

// #include <QVTKOpenGLNativeWidget.h>
// #include <vtkSphereSource.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkActor.h>
// #include <vtkProperty.h>
// #include <vtkAutoInit.h>
// VTK_MODULE_INIT(vtkRenderingOpenGL2);
// VTK_MODULE_INIT(vtkInteractionStyle);

VtkPresetWidget::VtkPresetWidget(QWidget *parent)
    : QWidget{parent}
{
    label=new QLabel("",this);
}






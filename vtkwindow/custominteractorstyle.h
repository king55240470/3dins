#ifndef CUSTOMINTERACTORSTYLE_H
#define CUSTOMINTERACTORSTYLE_H

#include <QVector>

#include <vtkInteractionStyleModule.h>
#include <vtkCommand.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkGenericRenderWindowInteractor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPropPicker.h>
#include <vtkCellPicker.h>
#include <vtkPointSource.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkMath.h>
#include <vtkProperty.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>

class CustomInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static CustomInteractorStyle* New();
    vtkTypeMacro(CustomInteractorStyle, vtkInteractorStyleTrackballCamera);
    CustomInteractorStyle();

    // 设置渲染器
    void SetRenderer(vtkRenderer* renderer) {
        this->renderer = renderer;
    }

    // 管理鼠标事件


private:
    vtkSmartPointer<vtkRenderer> renderer; // 渲染器成员

};

#endif // CUSTOMINTERACTORSTYLE_H

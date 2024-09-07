#include "vtkpresetwidget.h"

VtkPresetWidget::VtkPresetWidget(QWidget *parent)
    : QWidget{parent}
{
    label=new QLabel("预置的图形显示",this);
}

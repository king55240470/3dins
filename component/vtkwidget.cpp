#include "vtkwidget.h"

VtkWidget::VtkWidget(QWidget *parent)
    : QWidget{parent}
{
    label=new QLabel("label",this);
}

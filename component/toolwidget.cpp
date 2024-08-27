#include "toolwidget.h"

ToolWidget::ToolWidget(QWidget *parent)
    : QWidget{parent}
{
    label=new QLabel("label",this);
}

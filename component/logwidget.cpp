#include "logwidget.h"

LogWidget::LogWidget(QWidget *parent)
    : QWidget{parent}
{
    label=new QLabel("label",this);
}

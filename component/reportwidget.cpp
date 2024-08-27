#include "reportwidget.h"

ReportWidget::ReportWidget(QWidget *parent)
    : QWidget{parent}
{
    label=new QLabel("label",this);
}

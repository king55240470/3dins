#include "elementlistwidget.h"

ElementListWidget::ElementListWidget(QWidget *parent)
    : QWidget{parent}
{
    label=new QLabel("element list",this);
}

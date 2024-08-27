#include "unknownwidget.h"

UnknownWidget::UnknownWidget(QWidget *parent)
    : QWidget{parent}
{
    label=new QLabel("label",this);
}

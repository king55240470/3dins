#include "vtkwindowreportwidget.h"
#include <QTextEdit>
#include <QListWidget>
VtkWindowReportWidget::VtkWindowReportWidget(QWidget *parent)
    : QWidget{parent}
{
    QTextEdit *textedit=new QTextEdit("v",this);
    QListWidget *label=new QListWidget(this);
    label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

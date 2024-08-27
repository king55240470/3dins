#ifndef VTKWIDGET_H
#define VTKWIDGET_H

#include <QWidget>
#include <QLabel>

class VtkWidget : public QWidget
{
    Q_OBJECT
public:
    explicit VtkWidget(QWidget *parent = nullptr);
private:
    QLabel *label;
signals:
};

#endif // VTKWIDGET_H

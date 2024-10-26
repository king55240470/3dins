#ifndef VTKPRESETWIDGET_H
#define VTKPRESETWIDGET_H

#include <QLabel>


class VtkPresetWidget : public QWidget
{
    Q_OBJECT
public:
    explicit VtkPresetWidget(QWidget *parent = nullptr);

private:
    QLabel* label;

signals:
};

#endif // VTKPRESETWIDGET_H

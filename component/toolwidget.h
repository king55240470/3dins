#ifndef TOOLWIDGET_H
#define TOOLWIDGET_H

#include <QWidget>
#include <QLabel>

class ToolWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ToolWidget(QWidget *parent = nullptr);
private:
    QLabel *label;
signals:
};

#endif // TOOLWIDGET_H

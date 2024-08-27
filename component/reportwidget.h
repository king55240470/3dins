#ifndef REPORTWIDGET_H
#define REPORTWIDGET_H

#include <QWidget>
#include <QLabel>

class ReportWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ReportWidget(QWidget *parent = nullptr);
private:
    QLabel *label;
signals:
};

#endif // REPORTWIDGET_H

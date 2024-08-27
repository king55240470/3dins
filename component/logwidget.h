#ifndef LOGWIDGET_H
#define LOGWIDGET_H

#include <QWidget>
#include <QLabel>

class LogWidget : public QWidget
{
    Q_OBJECT
public:
    explicit LogWidget(QWidget *parent = nullptr);
private:
    QLabel *label;
signals:
};

#endif // LOGWIDGET_H

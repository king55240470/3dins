#ifndef UNKNOWNWIDGET_H
#define UNKNOWNWIDGET_H

#include <QWidget>
#include <QLabel>

class UnknownWidget : public QWidget
{
    Q_OBJECT
public:
    explicit UnknownWidget(QWidget *parent = nullptr);
private:
    QLabel *label;
signals:
};

#endif // UNKNOWNWIDGET_H

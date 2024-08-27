#ifndef ELEMENTLISTWIDGET_H
#define ELEMENTLISTWIDGET_H

#include <QWidget>
#include <QLabel>

class ElementListWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ElementListWidget(QWidget *parent = nullptr);
private:
    QLabel *label;
signals:
};

#endif // ELEMENTLISTWIDGET_H

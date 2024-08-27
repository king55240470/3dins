#ifndef DATAWIDGET_H
#define DATAWIDGET_H

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QTableWidget>

class DataWidget : public QWidget
{
    Q_OBJECT
public:
    explicit DataWidget(QWidget *parent = nullptr);
private:
    QVBoxLayout *vlayout;
    QHBoxLayout *hlayout;
    QLabel *label1;
    QLabel *label2;
    QLabel *label3;
    QComboBox *box;
    QTableWidget *table;
signals:
};

#endif // DATAWIDGET_H

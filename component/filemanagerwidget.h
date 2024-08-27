#ifndef FILEMANAGERWIDGET_H
#define FILEMANAGERWIDGET_H

#include <QWidget>
#include <QLabel>

class FileManagerWidget : public QWidget
{
    Q_OBJECT
public:
    explicit FileManagerWidget(QWidget *parent = nullptr);
private:
    QLabel *label;
signals:
};

#endif // FILEMANAGERWIDGET_H

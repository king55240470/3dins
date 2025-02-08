#ifndef VTKPRESETWIDGET_H
#define VTKPRESETWIDGET_H

#include <QTreeWidget>
#include <QVBoxLayout>

class MainWindow;
class VtkPresetWidget : public QWidget
{
    Q_OBJECT
public:
    explicit VtkPresetWidget(QWidget *parent = nullptr);
    void setWidget(QString);
    void showContextMenu(const QPoint &);
    void deleteWidget();

private:
    MainWindow *m_pMainWin;
    QVBoxLayout *layout;
    QTreeWidget *treeWidget;
    QMenu *contextMenu;//右键菜单
    QAction *deleteAction;
signals:
};


#include <QLabel>

class MessageItemWidget : public QWidget
{
public:
    MessageItemWidget(const QString &time, const QString &a, QWidget *parent = nullptr)
        : QWidget(parent) {
        QVBoxLayout *messageLayout=new QVBoxLayout(this);

        QLabel *timeLabel = new QLabel(time);
        timeLabel->setStyleSheet("font-size: 8pt; color: gray; background-color: white;");

        QLabel *messageLabel = new QLabel(a);
        messageLabel->setStyleSheet("font-size: 10pt; color: black;");

        messageLayout->addWidget(timeLabel);
        messageLayout->addWidget(messageLabel);

        setAttribute(Qt::WA_TransparentForMouseEvents); // 透明化鼠标事件

        setLayout(messageLayout);  // 设置布局
    }
};

#endif // VTKPRESETWIDGET_H

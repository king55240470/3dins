#ifndef PROGRESSDIALOGS_H
#define PROGRESSDIALOGS_H

#include <QDialog>
#include <QProgressBar>
#include <QVBoxLayout>
class ProgressDialogs : public QDialog
{
    Q_OBJECT
public:
    ProgressDialogs(QWidget *parent = nullptr);
    QProgressBar*progressBar;
public slots:
    void updateProgress(int value) {
        progressBar->setValue(value);
        if (value >= 100) accept(); // 完成后自动关闭
    }
};

#endif // PROGRESSDIALOGS_H

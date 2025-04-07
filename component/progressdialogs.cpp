#include "progressdialogs.h"
ProgressDialogs::ProgressDialogs(QWidget *parent)
: QDialog(parent)
{
    // 弹窗基本设置
    setWindowTitle("正在加载");
    setFixedSize(300, 100);
    setWindowModality(Qt::ApplicationModal); // 阻塞其他窗口
    setWindowFlags(windowFlags() & ~Qt::WindowCloseButtonHint); // 隐藏关闭按钮

    // 创建进度条
    progressBar = new QProgressBar(this);
    progressBar->setRange(0, 100);
    progressBar->setValue(0);
    progressBar->setTextVisible(true);

    // 布局设置
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(progressBar);
}

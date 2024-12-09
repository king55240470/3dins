#include "myftp.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QFormLayout>
myFTP::myFTP(QWidget *parent)
    : QWidget{parent}
{

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl=curl_easy_init();
    setWindowTitle("FTP文件传输");

    // 创建布局
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    // 创建表单布局
    QFormLayout *formLayout = new QFormLayout();
    mainLayout->addLayout(formLayout);

    // 创建用户名和密码输入框
    usernameLineEdit = new QLineEdit(this);
    passwordLineEdit = new QLineEdit(this);
    passwordLineEdit->setEchoMode(QLineEdit::Password); // 密码输入框隐藏输入内容

    // 将输入框添加到表单布局
    formLayout->addRow(new QLabel("用户名:", this), usernameLineEdit);
    formLayout->addRow(new QLabel("密码:", this), passwordLineEdit);

    // 创建按钮
    okButton = new QPushButton("确定", this);
    uploadButton = new QPushButton("上传", this);
    downloadButton = new QPushButton("下载", this);

    // 创建水平布局用于放置按钮
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(uploadButton);
    buttonLayout->addWidget(downloadButton);

    // 将按钮布局添加到主布局
    mainLayout->addLayout(buttonLayout);

    // 连接信号和槽
    connect(okButton, &QPushButton::clicked, this, &myFTP::on_okButton_clicked);
    connect(uploadButton, &QPushButton::clicked, this, &myFTP::on_uploadButton_clicked);
    connect(downloadButton, &QPushButton::clicked, this, &myFTP::on_downloadButton_clicked);
    if(curl){    curl_easy_cleanup(curl);
        curl=nullptr;}

    curl_global_cleanup();  // 清理全局资源
}

void myFTP::uploadFile(const QString &ftpUrl, const QString &filePath, const QString &username, const QString &password)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadWrite)) {
        qWarning() << "Failed to open file:" << filePath;
        return;
    }
    // 重置文件指针
    //file.seek(0);

    // 设置上传的目标 URL
    curl_easy_setopt(curl, CURLOPT_URL, ftpUrl.toStdString().c_str());

    // 设置用户名和密码
    curl_easy_setopt(curl, CURLOPT_USERPWD, QString("%1:%2").arg(username, password).toStdString().c_str());

    // 设置上传文件
    curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

    curl_easy_setopt(curl, CURLOPT_READFUNCTION, read_callback);

    curl_easy_setopt(curl, CURLOPT_READDATA, &file);
    // 开启详细调试信息（可选）
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
    // 执行上传
    CURLcode res = curl_easy_perform(curl);
    file.close();
    if (res != CURLE_OK) {
        qWarning() << "FTP upload failed:" << curl_easy_strerror(res);
    }


}

/*void myFTP::downloadFile(const QString &ftpUrl, const QString &filePath, const QString &username, const QString &password)
{
    if (!curl) return;


    QFile file(filePath);
    // 尝试以只写方式打开文件，若文件不存在则创建它
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        qWarning() << "Failed to open file for download:" << filePath;
        return;
    }

    // 设置目标 URL
    curl_easy_setopt(curl, CURLOPT_URL, ftpUrl.toStdString().c_str());

    // 设置用户名和密码
    curl_easy_setopt(curl, CURLOPT_USERPWD, QString("%1:%2").arg(username, password).toStdString().c_str());

    // 设置写入回调
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &file);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeData);

    // 开启详细日志
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);

    // 执行下载
    CURLcode res = curl_easy_perform(curl);
    file.close();

    if (res != CURLE_OK) {
        qWarning() << "Download failed:" << curl_easy_strerror(res);
    } else {
        qDebug() << "File downloaded successfully to" << filePath;
    }
}*/

void myFTP::downloadFiles(const QString &ftpUrl, const QString &localDir, const QString &username, const QString &password)
{
    QStringList fileList; // 存储文件列表
    listFTPFiles(ftpUrl, username, password, fileList); // 获取文件列表

    QDir dir(localDir); // 创建QDir对象，用于操作本地目录
    if (!dir.exists()) {
        dir.mkpath(localDir); // 如果目录不存在，则创建目录
    }
    qDebug()<<fileList.size();
    for (const QString &fileName : fileList) { // 遍历文件列表
        QString remoteFilePath = ftpUrl  + fileName; // 构造远程文件路径
        QString localFilePath = QDir(localDir).filePath(fileName); // 构造本地文件路径
        QFile file(localFilePath); // 创建QFile对象，用于写入下载的数据
        if (file.open(QIODevice::WriteOnly)) { // 打开文件以写入
            curl_easy_setopt(curl, CURLOPT_URL, remoteFilePath.toStdString().c_str()); // 设置远程文件URL
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeData); // 设置写数据回调函数
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &file); // 设置回调函数的userdata

            CURLcode res = curl_easy_perform(curl); // 执行CURL操作
            file.close(); // 关闭文件

            if (res != CURLE_OK) {
                qDebug() << "Download failed:" << curl_easy_strerror(res); // 输出错误信息
            } else {
                qDebug() << "File downloaded successfully to" << localFilePath; // 输出成功信息
            }
        }
    }

}

void myFTP::syncLocalWithFTP(const QString &ftpUrl, const QString &localDir, const QString &username, const QString &password)
{
    QStringList localFiles = QDir(localDir).entryList(QDir::Files); // 列出本地文件
    QStringList serverFileslist;

    listFTPFiles(ftpUrl, username, password, serverFileslist); // 列出FTP文件

    for (const QString &file : serverFileslist) {
        if (!localFiles.contains(file)) { // 比较文件
            QString fileToDownload = ftpUrl + file; // FTP 文件完整URL
            QString localFilePath = QDir(localDir).filePath(file); // 本地文件路径
            //downloadFile(fileToDownload, localFilePath, username, password); // 下载缺失的文件
        }
    }
}

void myFTP::setftpUrl(QString ftpurl)
{
    ftpUrl=ftpurl;
}

void myFTP::setfilePath(QString path)
{
    filePath=path;
}

void myFTP::setusername(QString name)
{
    username=name;
}

void myFTP::setpassword(QString p)
{
    password=p;
}

void myFTP::on_okButton_clicked()
{
    setusername(usernameLineEdit->text());
    setpassword(passwordLineEdit->text());
}

void myFTP::on_uploadButton_clicked()
{

}

void myFTP::on_downloadButton_clicked()
{

}

size_t myFTP::writeData(void *buffer, size_t size, size_t nmemb, void *userp)
{
    QFile *file = static_cast<QFile *>(userp);
    return file->write(static_cast<const char *>(buffer), size * nmemb);
}

size_t myFTP::read_callback(char *buffer, size_t size, size_t nitems, void *userdata)
{
    QFile *file = reinterpret_cast<QFile*>(userdata);
    qint64 retcode = file->read(buffer, size * nitems);
    if (retcode == -1) {
        return 0; // 到达文件末尾或发生错误
    }
    return static_cast<size_t>(retcode);
}

void myFTP::listFTPFiles(const QString &ftpUrl, const QString &username, const QString &password, QStringList &fileList)
{
    curl_easy_setopt(curl, CURLOPT_URL, ftpUrl.toStdString().c_str()); // 设置FTP URL
    curl_easy_setopt(curl, CURLOPT_USERPWD, QString("%1:%2").arg(username, password).toStdString().c_str()); // 设置用户名和密码
    qDebug()<<"list";
    std::string filelist; // 用于存储文件列表的字符串
    // 设置字符串回调函数，用于接收FTP服务器返回的数据
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
    // 设置回调函数的userdata
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &filelist);
    qDebug()<<"1";
    CURLcode res = curl_easy_perform(curl); // 执行CURL操作
    qDebug()<<"2";
    if (res != CURLE_OK) {
        qDebug() << "FTP list failed:" << curl_easy_strerror(res); // 输出错误信息
    } else {
        // 解析文件列表
        QStringList list = QString::fromStdString(filelist).split("\r", Qt::SkipEmptyParts);
        for (const QString &line : list) {
            // 这里假设文件名在行的末尾，通过分隔符来提取
            QString trimmedLine = line.trimmed(); // 去掉行首和行尾的空白
            QStringList parts = trimmedLine.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts); // 按空白字符分隔

            if (parts.size() > 3) { // 确保有日期、时间、文件大小和文件名
                QString filename = parts.last(); // 最后一个部分是文件名
                fileList << filename; // 添加到列表中
            }
        }
        qDebug()<<fileList;
    }
}

size_t myFTP::write_callback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

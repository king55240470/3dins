#ifndef MYFTP_H
#define MYFTP_H
#include <curl/curl.h> // 包含libcurl的头文件
#include <QFile>
#include <QSettings>
#include <QUrl>
#include <QDebug>
#include <QObject>
#include <QDir>
#include <QRegularExpression>
#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
class myFTP: public QWidget
{
    Q_OBJECT
public:
    explicit myFTP(QWidget *parent = nullptr);
    void uploadFile(const QString &ftpUrl, const QString &filePath, const QString &username, const QString &password);
    //void downloadFile(const QString &ftpUrl, const QString &filePath, const QString &username, const QString &password);
    void downloadFiles(const QString &ftpUrl, const QString &localDir, const QString &username, const QString &password);//下载所有文件
    void syncLocalWithFTP(const QString &ftpUrl, const QString &localDir, const QString &username, const QString &password);//比较本地与ftp上的文件，下载缺少的
    void setftpUrl(QString ftpurl);
    void setfilePath(QString path);
    void setusername(QString name);
    void setpassword(QString p);
    void on_okButton_clicked();
    void on_uploadButton_clicked();
    void on_downloadButton_clicked();

private:
    CURL *curl; // libcurl的easy接口句柄
    QSettings *settings;//配置ini文件
    QString ftpUrl;
    QString filePath;
    QString username;
    QString password;
    QLineEdit *usernameLineEdit;
    QLineEdit *passwordLineEdit;
    QPushButton *okButton;
    QPushButton *uploadButton;
    QPushButton *downloadButton;
    static size_t writeData(void *buffer, size_t size, size_t nmemb, void *userp);//下载
    static size_t read_callback(char *buffer, size_t size, size_t nitems, void *userdata);//写入
    void listFTPFiles(const QString &ftpUrl, const QString &username, const QString &password, QStringList &fileList);//获得ftp列表
    static size_t write_callback(void *contents, size_t size, size_t nmemb, void *userp);
};

#endif // MYFTP_H

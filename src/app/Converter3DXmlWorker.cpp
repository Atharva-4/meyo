#include "Converter3DXmlWorker.h"
#include <QProcess>
#include <QFileInfo>
#include <QFile>

namespace Mayo {
    Converter3DXmlWorker::Converter3DXmlWorker(
        const QString& input3dxml,
        const QString& exePath,
        QObject* parent)
        : QThread(parent)
        , m_input3dxml(input3dxml)
        , m_exePath(exePath)
    {
    }

    void Converter3DXmlWorker::run()
    {
        // Same logic as your original execute() — but now runs in background
        QProcess process;
        process.setWorkingDirectory(QFileInfo(m_input3dxml).absolutePath());
        process.start(m_exePath, QStringList() << m_input3dxml);
        process.waitForFinished(600000); // 10 minutes — same as yours

        const QString zipPath = QFileInfo(m_input3dxml).absolutePath() + "/" +
            QFileInfo(m_input3dxml).completeBaseName() + ".zip";

        if (!QFile::exists(zipPath)) {
            QString errMsg = QString::fromLocal8Bit(process.readAllStandardError());
            emit finished(false, QString(), errMsg);
            return;
        }

        emit finished(true, zipPath, QString());
    }
}
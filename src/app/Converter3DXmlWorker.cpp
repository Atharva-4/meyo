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
        QProcess process;
        process.setWorkingDirectory(QFileInfo(m_input3dxml).absolutePath());
        process.start(m_exePath, QStringList() << m_input3dxml);

        if (!process.waitForStarted(15000)) {
            emit finished(false, QString(), process.errorString());
            return;
        }

        if (!process.waitForFinished(600000)) {
            process.kill();
            process.waitForFinished();
            emit finished(false, QString(), tr("Converter timed out after 10 minutes."));
            return;
        }

        const QString zipPath = QFileInfo(m_input3dxml).absolutePath() + "/" +
            QFileInfo(m_input3dxml).completeBaseName() + ".zip";

        if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0 || !QFile::exists(zipPath)) {
            QString errMsg = QString::fromLocal8Bit(process.readAllStandardError()).trimmed();
            if (errMsg.isEmpty())
                errMsg = QString::fromLocal8Bit(process.readAllStandardOutput()).trimmed();
            if (errMsg.isEmpty())
                errMsg = tr("Converter exited with code %1.").arg(process.exitCode());
            emit finished(false, QString(), errMsg);
            return;
        }

        emit finished(true, zipPath, QString());
    }
}
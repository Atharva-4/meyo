#pragma once
#include <QThread>
#include <QString>
namespace Mayo {

    class Converter3DXmlWorker : public QThread
    {
        Q_OBJECT
    public:
        explicit Converter3DXmlWorker(
            const QString& input3dxml,
            const QString& exePath,
            QObject* parent = nullptr);

    signals:
        void finished(bool success, const QString& zipPath, const QString& errorMsg);

    protected:
        void run() override;

    private:
        QString m_input3dxml;
        QString m_exePath;
    };
}
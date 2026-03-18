#include "commands_koshika.h"

#include "../gui/gui_document.h"
#include "../gui/v3d_view_controller.h"
#include "widget_message_indicator.h"
#include <QAction>
#include <QtWidgets/QDialog>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFileDialog>

#include <QFileInfo> // tostdstring
#include <QApplication>
#include <QFile>
#include <QMessageBox>

#include <QThread>// holeFilling using Thread

#include <QtCore/QSignalBlocker>

#include <QListWidget>

#include "Converter3DXmlWorker.h"
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkReply>
#include <QtNetwork/QNetworkRequest>
#include <QEventLoop>
#include <QTimer>
#include <QUrl>

#include <QCoreApplication>

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <Graphic3d_ClipPlane.hxx>

#include <QInputDialog>
#include <QLineEdit>

#include <algorithm>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSlider>

namespace Mayo {

namespace {
bool g_is3dXmlConversionRunning = false;
}

///////////////////////////////////////////////////////////////////////////////
//
//covertor 3dxml
//
///////////////////////////////////////////////////////////////////////////////

CommandConvert3DXML::CommandConvert3DXML(IAppContext* context)
    : Command(context)
{
    auto action = new QAction(this);
    action->setText(Command::tr("Convert 3DXML to OBJ"));
    action->setToolTip(Command::tr("Convert a 3DXML file to OBJ format"));
    this->setAction(action);
}

void CommandConvert3DXML::execute()
{
    QWidget* parent = this->widgetMain();

    if (g_is3dXmlConversionRunning) {
        WidgetMessageIndicator::showInfo(tr("3DXML conversion is already running..."), parent);
        return;
    }

    QNetworkAccessManager networkManager;
    QNetworkReply* reply = networkManager.head(QNetworkRequest(QUrl("http://clients3.google.com/generate_204")));
    QEventLoop networkLoop;
    QTimer timeoutTimer;
    timeoutTimer.setSingleShot(true);

    connect(&timeoutTimer, &QTimer::timeout, &networkLoop, &QEventLoop::quit);
    connect(reply, &QNetworkReply::finished, &networkLoop, &QEventLoop::quit);

    timeoutTimer.start(5000);
    networkLoop.exec();

    const bool timedOut = !reply->isFinished();
    if (timedOut)
        reply->abort();

    const bool hasInternet = !timedOut && reply->error() == QNetworkReply::NoError;
    reply->deleteLater();

    if (!hasInternet) {
        QMessageBox::warning(parent, tr("No Internet"),
            tr("Internet connection is required for 3DXML conversion.\n"
               "Please check your connection and try again."));
        return;
    }

    const QString input3dxml = QFileDialog::getOpenFileName(
        parent, tr("Select 3DXML File"), QString(),
        tr("3DXML Files (*.3dxml);;All Files (*)")
    );
    if (input3dxml.isEmpty())
        return;

    const qint64 maxSize = 20LL * 1024 * 1024;
    const qint64 fileSize = QFileInfo(input3dxml).size();
    if (fileSize > maxSize) {
        QMessageBox::warning(parent, tr("File Too Large"),
            tr("The selected file is %1 MB.\n"
                "Maximum allowed size is 20 MB.\n\n"
                "Please select a smaller file.")
            .arg(QString::number(fileSize / (1024.0 * 1024.0), 'f', 1)));
        return;
    }

    const QString appDir = QCoreApplication::applicationDirPath();
    const QString exePath = appDir + "/3dxml_converter/3dxml_converter.exe";
    const QString zipPath = QFileInfo(input3dxml).absolutePath() + "/" +
        QFileInfo(input3dxml).completeBaseName() + ".zip";

    QFile::remove(zipPath);

    QMessageBox::information(parent, tr("3DXML Converter"),
        tr("Conversion will start now.\n\n"
            "NOTE: First time may take several minutes.\n"
            "Please wait until the success message appears.\n"
            "Do NOT close the application!"));

    g_is3dXmlConversionRunning = true;
    this->action()->setEnabled(false);
    QApplication::setOverrideCursor(Qt::BusyCursor);
    WidgetMessageIndicator::showInfo(tr("3DXML conversion started..."), parent);

    auto* worker = new Converter3DXmlWorker(input3dxml, exePath, parent);

    connect(worker, &Converter3DXmlWorker::finished,
        parent, [this, parent](bool success, const QString& outputZipPath, const QString& errMsg) {
            g_is3dXmlConversionRunning = false;
            this->action()->setEnabled(true);
            QApplication::restoreOverrideCursor();

            if (success) {
                WidgetMessageIndicator::showInfo(tr("3DXML conversion completed"), parent);
                QMessageBox::information(parent, tr("3DXML Converter"),
                    tr("Conversion complete!\n\nZIP saved to:\n%1\n\nRight-click → Extract All to get OBJ.")
                    .arg(outputZipPath));
            }
            else {
                QMessageBox::critical(parent, tr("3DXML Converter"),
                    tr("Conversion failed!\n%1").arg(errMsg));
            }
        });

    connect(worker, &QThread::finished, worker, &QObject::deleteLater);
    worker->start();
}

} // namespace Mayo

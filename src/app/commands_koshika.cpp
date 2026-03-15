#include "commands_koshika.h"

#include "../gui/gui_document.h"
#include "../gui/v3d_view_controller.h"
#include <QAction>
#include <QtWidgets/QDialog>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QProgressDialog> // Add this include at the top with other QtWidgets includes

#include <QFileInfo> // tostdstring
#include <QMessageBox>

#include <QThread>// holeFilling using Thread

#include <QtCore/QSignalBlocker>

#include <QListWidget>

#include <QProcessEnvironment>// for 3dxmcovertor
#include"Converter3DXmlWorker.h"
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkReply>      // ← ADD THIS
#include <QtNetwork/QNetworkRequest>    // ← ADD THIS
#include <QEventLoop>
#include <QUrl>                         

#include <QEventLoop>

#include <QCoreApplication>
#include <QProcess>

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

    // --- 1. Internet check ---

    QNetworkAccessManager nam;
    QNetworkReply* reply = nam.get(QNetworkRequest(QUrl("http://www.google.com")));
    QEventLoop loop;
    QObject::connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
    loop.exec();

    if (reply->error() != QNetworkReply::NoError) {
        QMessageBox::warning(parent, tr("No Internet"),
            tr("Internet connection is required for 3DXML conversion.\n"
                "Please check your connection and try again."));
        reply->deleteLater();
        return;
    }
    reply->deleteLater();

    // --- 2. File picker ---
    const QString input3dxml = QFileDialog::getOpenFileName(
        parent, tr("Select 3DXML File"), QString(),
        tr("3DXML Files (*.3dxml);;All Files (*)")
    );
    if (input3dxml.isEmpty())
        return;

    // --- 3. File size check (20 MB limit) ---
    const qint64 maxSize = 20LL * 1024 * 1024; // 20 MB in bytes
    const qint64 fileSize = QFileInfo(input3dxml).size();
    if (fileSize > maxSize) {
        QMessageBox::warning(parent, tr("File Too Large"),
            tr("The selected file is %1 MB.\n"
                "Maximum allowed size is 20 MB.\n\n"
                "Please select a smaller file.")
            .arg(QString::number(fileSize / (1024.0 * 1024.0), 'f', 1)));
        return;
    }

    // --- rest of your execute() continues here ---
    const QString appDir = QCoreApplication::applicationDirPath();
    const QString exePath = appDir + "/3dxml_converter/3dxml_converter.exe";

    QMessageBox::information(parent, tr("3DXML Converter"),
        tr("Conversion will start now.\n\n"
            "NOTE: First time may take several minutes.\n"
            "Please wait until the success message appears.\n"
            "Do NOT close the application!"));

    auto* progress = new QProgressDialog(
        tr("Converting 3DXML... Please wait."), QString(), 0, 0, parent);
    progress->setWindowModality(Qt::WindowModal);
    progress->setCancelButton(nullptr);
    progress->setMinimumDuration(0);
    progress->show();

    auto* worker = new Converter3DXmlWorker(input3dxml, exePath, parent);

    connect(worker, &Converter3DXmlWorker::finished,
        parent, [parent, progress, worker](bool success, const QString& zipPath, const QString& errMsg) {
            progress->close();
            progress->deleteLater();
            worker->deleteLater();

            if (success) {
                QMessageBox::information(parent, tr("3DXML Converter"),
                    tr("Conversion complete!\n\nZIP saved to:\n%1\n\nRight-click → Extract All to get OBJ.")
                    .arg(zipPath));
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

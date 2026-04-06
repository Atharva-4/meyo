#include "commands_Conversion.h"

#include "../gui/gui_document.h"
#include "../gui/v3d_view_controller.h"

#include <QAction>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QThread>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>
#include <QApplication>

#include "MeshConverter.h"   // your readPLY/STL/OBJ + writePLY/STL/OBJ
#include "dialog_save_image_view.h"  

namespace Mayo {

    // ─────────────────────────────────────────────────────────────────────────────
    //  Constructor
    // ─────────────────────────────────────────────────────────────────────────────
    CommandConvertor::CommandConvertor(IAppContext* context)
        : Command(context)
    {
        auto* action = new QAction(this);
        action->setText(Command::tr("Convertor"));
        action->setToolTip(Command::tr("Convert the current mesh to another format (PLY / STL / OBJ)"));
        this->setAction(action);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  execute()
    // ─────────────────────────────────────────────────────────────────────────────
    void CommandConvertor::execute()
    {
        if (m_isRunning)
            return;

        // ── 1. Get currently open document ───────────────────────────────────
        GuiDocument* guiDoc = this->currentGuiDocument();
        if (!guiDoc) {
            QMessageBox::warning(this->widgetMain(),
                tr("Convertor"),
                tr("No document is currently open.\nPlease open a PLY, STL or OBJ file first."));
            return;
        }

        const FilePath filePath = guiDoc->document()->filePath();
        if (filePath.empty()) {
            QMessageBox::warning(this->widgetMain(),
                tr("Convertor"),
                tr("Current document has no file path associated with it."));
            return;
        }

        const QString inputPath = QString::fromStdString(filePath.u8string());
        const QString inputExt = QFileInfo(inputPath).suffix().toLower();
        const QString inputName = QFileInfo(inputPath).fileName();

        // ── 2. Build conversion list based on input extension ────────────────
        struct ConvOption {
            QString label;       // shown in combo box
            QString outFilter;   // QFileDialog filter
            QString outExt;      // default output extension  (with dot)
            QString mode;        // internal: "PLY2STL", "STL2PLY", etc.
        };

        QVector<ConvOption> options;

        if (inputExt == "ply") {
            options << ConvOption{ "PLY  →  STL  (Binary)",  "STL Files (*.stl)", ".stl", "PLY2STL_BIN" };
            options << ConvOption{ "PLY  →  STL  (ASCII)",   "STL Files (*.stl)", ".stl", "PLY2STL_ASCII" };
            options << ConvOption{ "PLY  →  OBJ",            "OBJ Files (*.obj)", ".obj", "PLY2OBJ" };
        }
        else if (inputExt == "stl") {
            options << ConvOption{ "STL  →  PLY",            "PLY Files (*.ply)", ".ply", "STL2PLY" };
            options << ConvOption{ "STL  →  OBJ",            "OBJ Files (*.obj)", ".obj", "STL2OBJ" };
            options << ConvOption{ "STL  →  STL  (to Binary)",  "STL Files (*.stl)", ".stl", "STL2STL_BIN" };
            options << ConvOption{ "STL  →  STL  (to ASCII)",   "STL Files (*.stl)", ".stl", "STL2STL_ASCII" };
        }
        else if (inputExt == "obj") {
            options << ConvOption{ "OBJ  →  STL  (Binary)",  "STL Files (*.stl)", ".stl", "OBJ2STL_BIN" };
            options << ConvOption{ "OBJ  →  STL  (ASCII)",   "STL Files (*.stl)", ".stl", "OBJ2STL_ASCII" };
            options << ConvOption{ "OBJ  →  PLY",            "PLY Files (*.ply)", ".ply", "OBJ2PLY" };
        }
        else {
            QMessageBox::warning(this->widgetMain(),
                tr("Convertor"),
                tr("File type '.%1' is not supported.\n"
                    "Only PLY, STL and OBJ files can be converted.").arg(inputExt));
            return;
        }

        // ── 3. Dialog: choose conversion ─────────────────────────────────────
        QDialog dlg(this->widgetMain());
        dlg.setWindowTitle(tr("Convertor"));
        dlg.setMinimumWidth(380);

        auto* labelInfo = new QLabel(
            tr("<b>Current file:</b> %1<br><br>Select the conversion you want:").arg(inputName),
            &dlg);
        labelInfo->setWordWrap(true);

        auto* comboConv = new QComboBox(&dlg);
        for (const auto& opt : options)
            comboConv->addItem(opt.label);

        auto* btnBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);

        auto* layout = new QVBoxLayout(&dlg);
        layout->setSpacing(10);
        layout->setContentsMargins(16, 16, 16, 16);
        layout->addWidget(labelInfo);
        layout->addWidget(comboConv);
        layout->addWidget(btnBox);

        QObject::connect(btnBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        QObject::connect(btnBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

        if (dlg.exec() != QDialog::Accepted)
            return;

        const ConvOption& chosen = options[comboConv->currentIndex()];

        // ── 4. Ask where to save ──────────────────────────────────────────────
        const QString defaultOut =
            QFileInfo(inputPath).absolutePath() + "/" +
            QFileInfo(inputPath).completeBaseName() +
            "_converted" + chosen.outExt;

        const QString outputPath = QFileDialog::getSaveFileName(
            this->widgetMain(),
            tr("Save converted file as"),
            defaultOut,
            chosen.outFilter + tr(";;All Files (*)")
        );

        if (outputPath.isEmpty())
            return;

        // ── 5. Run conversion on background thread ────────────────────────────
        m_isRunning = true;

        QThread* thread = new QThread(this);
        QWidget* parentWgt = this->widgetMain();
        const QString inPath = inputPath;
        const QString outPath = outputPath;
        const QString mode = chosen.mode;

        connect(thread, &QThread::started, this, [=]() {

            bool ok = false;

            // ── reader ────────────────────────────────────────────────────────
            bool readOk = false;
            if (mode.startsWith("PLY")) readOk = readPLY(inPath.toStdString());
            else if (mode.startsWith("STL")) readOk = readSTL(inPath.toStdString());
            else if (mode.startsWith("OBJ")) readOk = readOBJ(inPath.toStdString());

            // ── writer ────────────────────────────────────────────────────────
            if (readOk) {
                if (mode.endsWith("PLY"))       ok = writePLY(outPath.toStdString());
                else if (mode.endsWith("OBJ"))       ok = writeOBJ(outPath.toStdString());
                else if (mode.endsWith("STL_BIN"))   ok = writeSTLBinary(outPath.toStdString());
                else if (mode.endsWith("STL_ASCII")) ok = writeSTLAscii(outPath.toStdString());
            }

            // ── report result on GUI thread ───────────────────────────────────
            const bool success = ok;
            QMetaObject::invokeMethod(qApp, [=]() {
                if (success) {
                    QMessageBox::information(parentWgt,
                        tr("Convertor"),
                        tr("Conversion successful!\n\nSaved to:\n%1").arg(outPath));
                }
                else {
                    QMessageBox::critical(parentWgt,
                        tr("Convertor"),
                        tr("Conversion failed.\n"
                            "Please check that the input file is a valid mesh file."));
                }
                }, Qt::QueuedConnection);

            // quit thread when done
            QMetaObject::invokeMethod(thread, "quit", Qt::QueuedConnection);
            });

        connect(thread, &QThread::finished, this, [this]() {
            m_isRunning = false;
            });
        connect(thread, &QThread::finished, thread, &QObject::deleteLater);

        thread->start();
    }

} // namespace Mayo

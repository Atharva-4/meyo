#include "commands_koshika.h"

#include "../gui/gui_document.h"
#include "../gui/v3d_view_controller.h"
#include <QAction>
#include <QtWidgets/QDialog>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFileDialog>

#include <QFileInfo> // tostdstring
#include "STLCutter.h"   // your algorithm
#include "StlHoleFilling.h" // your header in Mayo namespace
#include <QMessageBox>

#include <QThread>// holeFilling using Thread
#include "StlHoleFillingWorker.h"// worker class for hole filling

#include "STLMerger.h"// Stl mergeing

#include "StlCuttingWorker.h" // worker class for cutting
#include <QtCore/QSignalBlocker>

//#include "HoleFilling.h" //for hole filling selection
#include "HoleFillingUtils.h"
#include <QListWidget>

//highlight the holes
#include <AIS_Shape.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <TopoDS_Wire.hxx>
#include <Quantity_Color.hxx>
//#include"StlHoleFilling.cpp"

#include <QProcessEnvironment>// for 3dxmcovertor



#include <QCoreApplication>
#include <QProcess>

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <Graphic3d_ClipPlane.hxx>

#include "StlHoleFillingSelectedWorker.h"
#include <QInputDialog>
#include <QLineEdit>

#include <algorithm>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSlider>

//simplification

#define _CRT_SECURE_NO_WARNINGS


#include "../gui/gui_document.h"

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <TopoDS_Compound.hxx>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

// ── Your original mesh-simplification headers ────────────────────────────
#include "mesh.h"
#include "pmesh.h"


namespace Mayo {

    CommandCutting::CommandCutting(IAppContext* context)
        : Command(context)
    {
        auto action = new QAction(this);
        action->setText(Command::tr("Split into two"));
        action->setToolTip(Command::tr("Split the current model into two parts using a plane"));
        this->setAction(action);
    }
    void CommandCutting::execute()
    {
        if (m_isRunning)
            return;

        GuiDocument* guiDoc = this->currentGuiDocument();
        if (!guiDoc)
            return;

        const FilePath filePath = guiDoc->document()->filePath();
        if (filePath.empty())
            return;

        const QString inputPath = QString::fromStdString(filePath.u8string());

        STLCutter cutter;
        std::vector<Facet> facets = cutter.loadSTL(inputPath.toStdString());
        if (facets.empty()) {
            QMessageBox::warning(this->widgetMain(), tr("Split into two"), tr("Unable to load STL facets."));
            return;
        }

        // ---------------------------
        // 1. Plane selection + position dialog
        // ---------------------------
        QDialog dlg(this->widgetMain());
        dlg.setWindowTitle(tr("Splitting Plane"));

        auto comboPlane = new QComboBox(&dlg);
        comboPlane->addItem("YZ Plane");
        comboPlane->addItem("XZ Plane");
        comboPlane->addItem("XY Plane");

        auto sliderCutPos = new QSlider(Qt::Horizontal, &dlg);
        sliderCutPos->setRange(0, 1000);
        auto spinCutPos = new QDoubleSpinBox(&dlg);
        spinCutPos->setDecimals(4);
        auto checkPreview = new QCheckBox(tr("Preview in viewport while moving slider"), &dlg);
        checkPreview->setChecked(true);

        auto clipPlanePreview = new Graphic3d_ClipPlane(gp_Pln(gp::Origin(), gp::DX()));
        clipPlanePreview->SetOn(false);
        clipPlanePreview->SetCapping(false); // only clip, don't show capping plane
        guiDoc->v3dView()->AddClipPlane(clipPlanePreview);

        auto setPreviewPlanePosition = [=](double pos) {
            const gp_Dir normal = clipPlanePreview->ToPlane().Axis().Direction();
            const gp_XYZ xyz = gp_Vec(normal).XYZ() * pos;
            clipPlanePreview->SetEquation(gp_Pln(gp_Pnt(xyz), normal));
            };

        auto updateAxisAndRange = [&]() {
            char axis = 'X';
            gp_Dir normal = gp::DX();
            if (comboPlane->currentText() == "XZ Plane") {
                axis = 'Y';
                normal = gp::DY();
            }
            else if (comboPlane->currentText() == "XY Plane") {
                axis = 'Z';
                normal = gp::DZ();
            }

            float minV = 0.f;
            float maxV = 0.f;
            cutter.getBounds(facets, axis, minV, maxV);
            const double span = maxV - minV;
            const double gap = span * 0.01;
            spinCutPos->setRange(minV - gap, maxV + gap);
            spinCutPos->setSingleStep(std::abs(maxV - minV) / 200.0);
            spinCutPos->setValue((minV + maxV) * 0.5);
            clipPlanePreview->SetEquation(gp_Pln(gp::Origin(), normal));
            setPreviewPlanePosition(spinCutPos->value());
            };

        QObject::connect(comboPlane, &QComboBox::currentTextChanged, &dlg, [=](const QString&) {
            updateAxisAndRange();
            });

        QObject::connect(spinCutPos, qOverload<double>(&QDoubleSpinBox::valueChanged), &dlg, [=](double value) {
            const double ratio = (value - spinCutPos->minimum()) /
                std::max(1e-9, spinCutPos->maximum() - spinCutPos->minimum());
            {
                QSignalBlocker blocker(sliderCutPos);
                sliderCutPos->setValue(qRound(ratio * sliderCutPos->maximum()));
            }

            if (checkPreview->isChecked()) {
                clipPlanePreview->SetOn(true);
                setPreviewPlanePosition(value);
                guiDoc->graphicsView().redraw();
            }
            });

        QObject::connect(sliderCutPos, &QSlider::valueChanged, &dlg, [=](int value) {
            const double ratio = double(value) / std::max(1, sliderCutPos->maximum());
            const double cutPos = spinCutPos->minimum() + ratio * (spinCutPos->maximum() - spinCutPos->minimum());
            QSignalBlocker blocker(spinCutPos);
            spinCutPos->setValue(cutPos);

            if (checkPreview->isChecked()) {
                clipPlanePreview->SetOn(true);
                setPreviewPlanePosition(cutPos);
                guiDoc->graphicsView().redraw();
            }
            });

        QObject::connect(checkPreview, &QCheckBox::toggled, &dlg, [=](bool on) {
            clipPlanePreview->SetOn(on);
            guiDoc->graphicsView().redraw();
            });

        auto btnBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            &dlg
        );

        auto layoutCutPos = new QHBoxLayout;
        layoutCutPos->addWidget(new QLabel(tr("Cut value"), &dlg));
        layoutCutPos->addWidget(spinCutPos);

        auto layout = new QVBoxLayout(&dlg);
        layout->addWidget(new QLabel(tr("Choose plane orientation"), &dlg));
        layout->addWidget(comboPlane);
        layout->addWidget(sliderCutPos);
        layout->addLayout(layoutCutPos);
        layout->addWidget(checkPreview);
        layout->addWidget(btnBox);

        QObject::connect(btnBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        QObject::connect(btnBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

        updateAxisAndRange();

        if (dlg.exec() != QDialog::Accepted) {
            guiDoc->v3dView()->RemoveClipPlane(clipPlanePreview);
            guiDoc->graphicsView().redraw();
            return;
        }

        // ---------------------------
        // 2. Plane enum -> axis
        // ---------------------------
        CommandCutting::CutPlane plane = CommandCutting::CutPlane::Z;

        const QString selected = comboPlane->currentText();
        if (selected == "YZ Plane") plane = CommandCutting::CutPlane::X;
        else if (selected == "XZ Plane") plane = CommandCutting::CutPlane::Y;
        else if (selected == "XY Plane") plane = CommandCutting::CutPlane::Z;

        char axis = 'Z';
        switch (plane) {
        case CommandCutting::CutPlane::X: axis = 'X'; break;
        case CommandCutting::CutPlane::Y: axis = 'Y'; break;
        case CommandCutting::CutPlane::Z: axis = 'Z'; break;
        }

        const float cutValue = static_cast<float>(spinCutPos->value());

        guiDoc->v3dView()->RemoveClipPlane(clipPlanePreview);
        guiDoc->graphicsView().redraw();

        // ------------------------------------------
        // 3. Ask user for save directory
        // ------------------------------------------
        QString dirPath = QFileDialog::getExistingDirectory(
            this->widgetMain(),
            tr("Select Folder to Save Cut STL Files")
        );

        if (dirPath.isEmpty())
            return;

        // Build output file paths
        const QString outAbove = dirPath + "/cut_1.stl";
        const QString outBelow = dirPath + "/cut_2.stl";

        m_isRunning = true;

        QThread* thread = new QThread(this);
        auto* worker = new StlCuttingWorker();
        worker->moveToThread(thread);

        connect(thread, &QThread::started, this, [=]() {
            QMetaObject::invokeMethod(worker, "process",
                Qt::QueuedConnection,
                Q_ARG(QString, inputPath),
                Q_ARG(QString, outAbove),
                Q_ARG(QString, outBelow),
                Q_ARG(char, axis),
                Q_ARG(float, cutValue));
            });

        connect(worker, &StlCuttingWorker::finished, this, [=]() {
            QMessageBox::information(
                this->widgetMain(),
                tr("Split into two"),
                tr("Cut STL files written to:\n%1\n%2").arg(outAbove, outBelow)
            );
            m_isRunning = false;
            thread->quit();
            });

        connect(worker, &StlCuttingWorker::error, this, [=](const QString& msg) {
            QMessageBox::critical(this->widgetMain(), tr("Split Error"), msg);
            m_isRunning = false;
            thread->quit();
            });

        connect(thread, &QThread::finished, worker, &QObject::deleteLater);
        connect(thread, &QThread::finished, thread, &QObject::deleteLater);

        thread->start();
    }
    /////////////////////////////////////////////////////////////////////
    //
    //Hole Filling Without Selection
    // 
    ////////////////////////////////////////////////////////////////////

    CommandHoleFillingFull::CommandHoleFillingFull(IAppContext* context)
        : Command(context)
    {
        auto action = new QAction(this);
        action->setText(tr("Fill Holes (All)"));
        action->setToolTip(tr("Fill all holes in an STL mesh (full fill)"));
        setAction(action);
        connect(action, &QAction::triggered, this, &CommandHoleFillingFull::execute);
    }

    void CommandHoleFillingFull::execute()
    {
        if (m_isRunning)
            return;

        m_isRunning = true;

        GuiDocument* guiDoc = this->currentGuiDocument();
        if (!guiDoc) {
            m_isRunning = false;
            return;
        }

        const FilePath filePath = guiDoc->document()->filePath();
        if (filePath.empty()) {
            m_isRunning = false;
            return;
        }

        QString inPath = QString::fromStdString(filePath.u8string());

        QWidget* parent = widgetMain();

        QString outPath = QFileDialog::getSaveFileName(
            parent,
            tr("Save repaired STL"),
            QFileInfo(inPath).completeBaseName() + "_repaired.stl",
            tr("STL Files (*.stl);;All Files (*)"));

        if (outPath.isEmpty()) {
            m_isRunning = false;
            return;
        }

        QThread* thread = new QThread(this);
        Mayo::StlHoleFillingWorker* worker = new Mayo::StlHoleFillingWorker();

        worker->moveToThread(thread);

        connect(thread, &QThread::started, this, [=]() {
            QMetaObject::invokeMethod(worker, "process",
                Qt::QueuedConnection,
                Q_ARG(QString, inPath),
                Q_ARG(QString, outPath));
            });

        connect(worker, &Mayo::StlHoleFillingWorker::finished,
            this, [=]() {
                QMessageBox::information(parent,
                    tr("Hole Filling"),
                    tr("Repaired STL written to:\n%1").arg(outPath));
                m_isRunning = false;  // reset here
                thread->quit();
            });

        connect(worker, &Mayo::StlHoleFillingWorker::error,
            this, [=](const QString& msg) {
                QMessageBox::critical(parent,
                    tr("Hole Filling Error"),
                    msg);
                m_isRunning = false;  // reset here
                thread->quit();
            });

        connect(thread, &QThread::finished, worker, &QObject::deleteLater);
        connect(thread, &QThread::finished, thread, &QObject::deleteLater);

        thread->start();
    }


    /////////////////////////////////////////////////////////////////////
    //
    //Hole Filling With Selection
    // 
    ////////////////////////////////////////////////////////////////////


    CommandHoleFillingSelected::CommandHoleFillingSelected(IAppContext* context)
        : Command(context)
    {
        auto action = new QAction(this);
        action->setText(tr("Fill Holes (Selected)"));
        action->setToolTip(tr("Fill only selected holes by index"));
        setAction(action);
        connect(action, &QAction::triggered, this, &CommandHoleFillingSelected::execute);
    }

    void CommandHoleFillingSelected::execute()
    {
        if (m_isRunning)
            return;

        GuiDocument* guiDoc = this->currentGuiDocument();
        if (!guiDoc)
            return;

        const FilePath filePath = guiDoc->document()->filePath();
        if (filePath.empty())
            return;

        const QString inPath = QString::fromStdString(filePath.u8string());
        QWidget* parent = widgetMain();

        int detectedHoleCount = -1;
        if (const auto holeCount = detectHoleCountFromStl(inPath.toStdString()))
            detectedHoleCount = static_cast<int>(*holeCount);


       
        QVector<int> selectedIds;
        if (detectedHoleCount < 0) {
    QMessageBox::warning(parent, tr("Error"), tr("Could not detect holes."));
    return;
    }
    else if (detectedHoleCount == 0) {
    QMessageBox::information(parent, tr("No Holes"), tr("No holes detected in this file."));
    return;
    }
        if (detectedHoleCount > 0) {
            QDialog dlg(parent);
            dlg.setWindowTitle(tr("Fill Holes (Selected)"));

            auto* layout = new QVBoxLayout(&dlg);
            auto* label = new QLabel(
                tr("Detected %1 hole(s). Click to select holes, then press Enter/OK.").arg(detectedHoleCount),
                &dlg
            );
            auto* list = new QListWidget(&dlg);
            list->setSelectionMode(QAbstractItemView::MultiSelection);
            for (int i = 0; i < detectedHoleCount; ++i)
                list->addItem(QString::number(i));

   
                // Insert immediately after populating 'list' (after the for loop that adds list items)

                        // Precompute hole boundaries (STL -> SurfaceMesh -> boundary loops)
                std::vector<std::vector<Mayo::Point>> holeBoundaries;
            {
                std::vector<Mayo::Triangles> triangles;
                if (Mayo::isBinarySTL(inPath.toStdString()))
                    Mayo::readBinarySTL(inPath.toStdString(), triangles);
                else
                    Mayo::readASCIISTL(inPath.toStdString(), triangles);

                if (!triangles.empty()) {
                    Mayo::SurfaceMesh mesh = Mayo::convertToSurfaceMesh(triangles);
                    holeBoundaries = Mayo::extractHoleBoundaries(mesh); // implemented above
                }
            }

            // Keep track of temporary highlight objects added to scene while dialog is open
            std::vector<Mayo::GraphicsObjectPtr> currentHighlights;

            // When the focused row changes show a temporary red wire for that hole
            QObject::connect(list, &QListWidget::currentRowChanged, &dlg,
                [guiDoc, &holeBoundaries, &currentHighlights](int row) {
                    // erase previous highlights
                    for (auto& h : currentHighlights) {
                        if (h) guiDoc->graphicsScene()->eraseObject(h);
                    }
                    currentHighlights.clear();
                    guiDoc->graphicsView().redraw();

                    if (row < 0 || row >= static_cast<int>(holeBoundaries.size()))
                        return;

                    const auto& loop = holeBoundaries[row];
                    if (loop.size() < 2)
                        return;

                    // build polygon wire from loop points
                    BRepBuilderAPI_MakePolygon poly;
                    for (const auto& p : loop) {
                        poly.Add(gp_Pnt(p.x(), p.y(), p.z()));
                    }
                    poly.Close();
                    TopoDS_Wire wire = poly.Wire();

                    auto* ais = new AIS_Shape(wire);
                    ais->SetColor(Quantity_NOC_RED);
                    ais->SetDisplayMode(AIS_WireFrame);

                    Mayo::GraphicsObjectPtr highlightObj = ais;
                    guiDoc->graphicsScene()->addObject(highlightObj, Mayo::GraphicsScene::AddObjectDisableSelectionMode);
                    guiDoc->graphicsView().redraw();
                    currentHighlights.push_back(highlightObj);
                });

            auto* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
            connect(buttonBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
            connect(buttonBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

            layout->addWidget(label);
            layout->addWidget(list);
            layout->addWidget(buttonBox);

            if (dlg.exec() != QDialog::Accepted)
                return;

            for (QListWidgetItem* item : list->selectedItems()) {
                bool okId = false;
                const int id = item->text().toInt(&okId);
                if (okId)
                    selectedIds.push_back(id);
            }
        }
        else {
            bool ok = false;
            const QString txtIds = QInputDialog::getText(
                parent,
                tr("Fill Holes (Selected)"),
                tr("Enter hole indices (comma separated, e.g. 0,2,5):"),
                QLineEdit::Normal,
                QString(),
                &ok
            );
            if (!ok || txtIds.trimmed().isEmpty())
                return;

            for (const QString& part : txtIds.split(',', Qt::SkipEmptyParts)) {
                bool idOk = false;
                const int id = part.trimmed().toInt(&idOk);
                if (idOk)
                    selectedIds.push_back(id);
            }
        }

        if (selectedIds.isEmpty()) {
            QMessageBox::warning(parent, tr("Fill Holes (Selected)"), tr("No valid hole index entered."));
            return;
        }

        const QString outPath = QFileDialog::getSaveFileName(
            parent,
            tr("Save repaired STL"),
            QFileInfo(inPath).completeBaseName() + "_selected_repaired.stl",
            tr("STL Files (*.stl);;All Files (*)")
        );
        if (outPath.isEmpty())
            return;

        QThread* thread = new QThread(this);
        auto* worker = new StlHoleFillingSelectedWorker();
        worker->moveToThread(thread);
        m_isRunning = true;

        connect(thread, &QThread::started, this, [=]() {
            QMetaObject::invokeMethod(worker, "process",
                Qt::QueuedConnection,
                Q_ARG(QString, inPath),
                Q_ARG(QString, outPath),
                Q_ARG(QVector<int>, selectedIds));
            });

        connect(worker, &StlHoleFillingSelectedWorker::finished, this, [=]() {
            QMessageBox::information(parent, tr("Fill Holes (Selected)"),
                tr("Repaired STL written to:\n%1").arg(outPath));
            m_isRunning = false;
            thread->quit();
            });

        connect(worker, &StlHoleFillingSelectedWorker::error, this, [=](const QString& msg) {
            QMessageBox::critical(parent, tr("Fill Holes (Selected) Error"), msg);
            m_isRunning = false;
            thread->quit();
            });

        connect(thread, &QThread::finished, this, [this]() {
            m_isRunning = false; // extra safety
            });
        connect(thread, &QThread::finished, worker, &QObject::deleteLater);
        connect(thread, &QThread::finished, thread, &QObject::deleteLater);

        thread->start();
    }


    


///////////////////////////////////////////////////////////////////
//
// STL Merge Command
//
///////////////////////////////////////////////////////////////////

    CommandMergeSTL::CommandMergeSTL(IAppContext* context)
        : Command(context)
    {
        auto action = new QAction(this);
        action->setText(Command::tr("Merge STL Files"));
        action->setToolTip(Command::tr("Merge multiple STL files into a single mesh"));
        this->setAction(action);
    }

    void CommandMergeSTL::execute()
    {
        QWidget* parent = this->widgetMain();

        const QStringList inPaths = QFileDialog::getOpenFileNames(
            parent,
            tr("Select STL files to merge"),
            QString(),
            tr("STL Files (*.stl);;All Files (*)")
        );

        if (inPaths.size() < 2) {
            if (!inPaths.isEmpty()) {
                QMessageBox::information(parent, tr("Merge STL Files"), tr("Select at least two STL files."));
            }
            return;
        }

        const QString outPath = QFileDialog::getSaveFileName(
            parent,
            tr("Save merged STL"),
            "merged.stl",
            tr("STL Files (*.stl);;All Files (*)")
        );

        if (outPath.isEmpty())
            return;

        STLCutter cutter;
        std::vector<Facet> mergedFacets;

        for (const QString& inPath : inPaths) {
            const auto facets = cutter.loadSTL(inPath.toStdString());
            if (facets.empty()) {
                QMessageBox::warning(
                    parent,
                    tr("Merge STL Files"),
                    tr("Failed to load or empty STL:\n%1").arg(inPath)
                );
                return;
            }

            mergedFacets.insert(mergedFacets.end(), facets.begin(), facets.end());
        }

        cutter.saveSTL(outPath.toStdString(), mergedFacets);
        QMessageBox::information(
            parent,
            tr("Merge STL Files"),
            tr("Merged STL written to:\n%1").arg(outPath)
        );
    }

///////////////////////////////////////////////////////////////////
//
// Point to Surface Command
//
///////////////////////////////////////////////////////////////////


    CommandPointToSurfaceWithNormals::CommandPointToSurfaceWithNormals(IAppContext* context)
        : Command(context)
    {
        auto action = new QAction(this);
        action->setText(Command::tr("With Normal"));
        action->setToolTip(Command::tr("Point to Surface using normals - requires CUDA GPU"));
        this->setAction(action);
    }

    void CommandPointToSurfaceWithNormals::execute()
    {
        QWidget* parent = this->widgetMain();

        const QString inputPly = QFileDialog::getOpenFileName(
            parent, tr("Select Input PLY File"), QString(),
            tr("PLY Files (*.ply);;All Files (*)")
        );
        if (inputPly.isEmpty())
            return;

        const QString outputObj = QFileDialog::getSaveFileName(
            parent, tr("Save Output OBJ File"),
            QFileInfo(inputPly).completeBaseName() + "_surface.obj",
            tr("OBJ Files (*.obj);;All Files (*)")
        );
        if (outputObj.isEmpty())
            return;

        const QString appDir = QCoreApplication::applicationDirPath();

        const QString exePath = appDir + "/ply2objwithnormal/ply2objwithnormal.exe";

        QProcess process;
        process.setWorkingDirectory(appDir + "/ply2objwithnormal");
        process.start(exePath, QStringList() << inputPly << outputObj);
        process.waitForFinished(-1);

        if (process.exitCode() != 0) {
            QMessageBox::critical(parent, tr("Point to Surface (With Normals)"),
                tr("ply2objwithnormal.exe failed!\nMake sure CUDA GPU is available."));
            return;
        }

        QMessageBox::information(parent, tr("Point to Surface (With Normals)"),
            tr("Done!\nOutput saved to:\n%1").arg(outputObj));
    }

    ///////////////////////////////////////////////////////////////////
    // 
    // Point to Surface - Without Normals
    // 
    ///////////////////////////////////////////////////////////////////

    CommandPointToSurfaceWithoutNormals::CommandPointToSurfaceWithoutNormals(IAppContext* context)
        : Command(context)
    {
        auto action = new QAction(this);
        action->setText(Command::tr("Without Normals"));
        action->setToolTip(Command::tr("Point to Surface without normals - no CUDA required"));
        this->setAction(action);
    }

    void CommandPointToSurfaceWithoutNormals::execute()
    {
        QWidget* parent = this->widgetMain();

        const QString inputPly = QFileDialog::getOpenFileName(
            parent, tr("Select Input PLY File"), QString(),
            tr("PLY Files (*.ply);;All Files (*)")
        );
        if (inputPly.isEmpty())
            return;

        const QString outputObj = QFileDialog::getSaveFileName(
            parent, tr("Save Output OBJ File"),
            QFileInfo(inputPly).completeBaseName() + "_surface.obj",
            tr("OBJ Files (*.obj);;All Files (*)")
        );
        if (outputObj.isEmpty())
            return;

        const QString appDir = QCoreApplication::applicationDirPath(); 
        const QString exePath = appDir + "/ply2objwithoutnormal/ply2objwithoutnormal.exe";

        QProcess process;
        process.setWorkingDirectory(appDir + "/ply2objwithoutnormal");
        process.start(exePath, QStringList() << inputPly << outputObj);
        process.waitForFinished(-1);

        if (process.exitCode() != 0) {
            QMessageBox::critical(parent, tr("Point to Surface (Without Normals)"),
                tr("ply2objwithoutnormal.exe failed!"));
            return;
        }

        QMessageBox::information(parent, tr("Point to Surface (Without Normals)"),
            tr("Done!\nOutput saved to:\n%1").arg(outputObj));
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

    const QString input3dxml = QFileDialog::getOpenFileName(
        parent, tr("Select 3DXML File"), QString(),
        tr("3DXML Files (*.3dxml);;All Files (*)")
    );
    if (input3dxml.isEmpty())
        return;

    const QString appDir = QCoreApplication::applicationDirPath();
    const QString exePath = appDir + "/3dxml_converter/3dxml_converter.exe";

    QMessageBox::information(parent, tr("3DXML Converter"),
        tr("Conversion will start now.\n\n"
            "NOTE: First time may take several minutes.\n"
            "Please wait until the success message appears.\n"
            "Do NOT close the application!"));

    QProcess process;
    process.setWorkingDirectory(QFileInfo(input3dxml).absolutePath());
    process.start(exePath, QStringList() << input3dxml);
    process.waitForFinished(600000); // 10 minutes

    const QString zipPath = QFileInfo(input3dxml).absolutePath() + "/" +
        QFileInfo(input3dxml).completeBaseName() + ".zip";

    if (!QFile::exists(zipPath)) {
        QString errMsg = QString::fromLocal8Bit(process.readAllStandardError());
        QMessageBox::critical(parent, tr("3DXML Converter"),
            tr("Conversion failed!\n%1").arg(errMsg));
        return;
    }

    QMessageBox::information(parent, tr("3DXML Converter"),
        tr("Conversion complete!\n\nZIP saved to:\n%1\n\nRight-click → Extract All to get OBJ.")
        .arg(zipPath));
}
//////////////////////////////////////////////////////////////////
//
// 
//
//////////////////////////////////////////////////////////////////



constexpr const char CommandSimplification::Name[];

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────
CommandSimplification::CommandSimplification(IAppContext* context)
    : Command(context)
{
    auto* action = new QAction(this);
    action->setText(Command::tr("Simplify Mesh"));
    action->setToolTip(Command::tr(
        "Reduce triangle count of the current mesh using progressive mesh simplification"));
    this->setAction(action);
}

// ─────────────────────────────────────────────────────────────────────────────
//  execute()
// ─────────────────────────────────────────────────────────────────────────────
void CommandSimplification::execute()
{
    if (m_isRunning)
        return;

    // ── 1. Get the currently open document ───────────────────────────────
    GuiDocument* guiDoc = this->currentGuiDocument();
    if (!guiDoc) {
        QMessageBox::warning(this->widgetMain(),
            tr("Simplify Mesh"),
            tr("No document is currently open.\n"
               "Please open an STL, PLY or OBJ file first."));
        return;
    }

    const FilePath filePath = guiDoc->document()->filePath();
    if (filePath.empty()) {
        QMessageBox::warning(this->widgetMain(),
            tr("Simplify Mesh"),
            tr("Current document has no file path associated with it."));
        return;
    }

    const QString inputPath = QString::fromStdString(filePath.u8string());
    const QString inputExt = QFileInfo(inputPath).suffix().toLower();
    const QString inputName = QFileInfo(inputPath).fileName();

    // Only support formats that Mesh can load
    if (inputExt != "stl" && inputExt != "ply" && inputExt != "obj") {
        QMessageBox::warning(this->widgetMain(),
            tr("Simplify Mesh"),
            tr("Unsupported file type: .%1\n"
               "Only STL, PLY and OBJ files can be simplified.").arg(inputExt));
        return;
    }

    // ── 2. Load mesh now so we can show real triangle count in dialog ─────
    // We load on the GUI thread here just to get the count — it's fast enough.
    // The heavy simplification work runs on the background thread later.
    QByteArray pathBytes = inputPath.toLocal8Bit();
    char* pathCStr = pathBytes.data();

    Mesh* previewMesh = new Mesh(pathCStr);
    if (previewMesh->getNumTriangles() == 0) {
        delete previewMesh;
        QMessageBox::critical(this->widgetMain(),
            tr("Simplify Mesh"),
            tr("Failed to load mesh from:\n%1\n\n"
               "File may be empty or corrupt.").arg(inputPath));
        return;
    }

    const int origTris = previewMesh->getNumTriangles();
    delete previewMesh; // free preview — we'll reload on the worker thread

    // ── 3. Dialog ─────────────────────────────────────────────────────────
    QDialog dlg(this->widgetMain());
    dlg.setWindowTitle(tr("Simplify Mesh"));
    dlg.setMinimumWidth(420);

    // Info label
    auto* labelFile = new QLabel(
        tr("<b>File:</b> %1<br><b>Original triangles:</b> %2")
            .arg(inputName)
            .arg(origTris),
        &dlg);
    labelFile->setWordWrap(true);

    // Edge-cost method
    auto* labelMethod = new QLabel(tr("Simplification method:"), &dlg);
    auto* comboMethod = new QComboBox(&dlg);
    comboMethod->addItem(tr("Quadric (best quality)"),        QVariant(int(PMesh::QUADRIC)));
    comboMethod->addItem(tr("Quadric weighted by tri area"),  QVariant(int(PMesh::QUADRICTRI)));
    comboMethod->addItem(tr("Melax"),                         QVariant(int(PMesh::MELAX)));
    comboMethod->addItem(tr("Shortest Edge (fastest)"),       QVariant(int(PMesh::SHORTEST)));
    comboMethod->setCurrentIndex(0);

    // Reduction slider
    auto* labelReduction = new QLabel(tr("Triangle reduction:"), &dlg);

    auto* slider = new QSlider(Qt::Horizontal, &dlg);
    slider->setRange(1, 99);
    slider->setValue(50);

    auto* spinBox = new QSpinBox(&dlg);
    spinBox->setRange(1, 99);
    spinBox->setValue(50);
    spinBox->setSuffix(" %");

    auto* labelResult = new QLabel(&dlg);
    auto updateResultLabel = [&]() {
        const int pct = spinBox->value();
        const int remaining = qMax(1, int(origTris * (1.0 - pct / 100.0)));
        labelResult->setText(
            tr("→  ~%1 triangles remaining  (%2% removed)")
                .arg(remaining).arg(pct));
    };
    updateResultLabel();

    // Sync slider ↔ spinbox
    QObject::connect(slider,  &QSlider::valueChanged,
                     spinBox, &QSpinBox::setValue);
    QObject::connect(spinBox, qOverload<int>(&QSpinBox::valueChanged),
                     slider,  &QSlider::setValue);
    QObject::connect(spinBox, qOverload<int>(&QSpinBox::valueChanged),
                     &dlg,    [&](int) { updateResultLabel(); });

    auto* sliderRow = new QHBoxLayout;
    sliderRow->addWidget(slider);
    sliderRow->addWidget(spinBox);

    auto* btnBox = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    QObject::connect(btnBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(btnBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    auto* layout = new QVBoxLayout(&dlg);
    layout->setSpacing(10);
    layout->setContentsMargins(16, 16, 16, 16);
    layout->addWidget(labelFile);
    layout->addSpacing(6);
    layout->addWidget(labelMethod);
    layout->addWidget(comboMethod);
    layout->addSpacing(6);
    layout->addWidget(labelReduction);
    layout->addLayout(sliderRow);
    layout->addWidget(labelResult);
    layout->addSpacing(6);
    layout->addWidget(btnBox);

    if (dlg.exec() != QDialog::Accepted)
        return;

    const int reductionPct = spinBox->value();
    const PMesh::EdgeCost edgeCost =
        static_cast<PMesh::EdgeCost>(comboMethod->currentData().toInt());

    // ── 4. Ask where to save ──────────────────────────────────────────────
    QString outFilter;
    if (inputExt == "stl") outFilter = tr("STL Files (*.stl)");
    else if (inputExt == "ply") outFilter = tr("PLY Files (*.ply)");
    else                        outFilter = tr("OBJ Files (*.obj)");

    const QString defaultOut =
        QFileInfo(inputPath).absolutePath() + "/" +
        QFileInfo(inputPath).completeBaseName() +
        QString("_simplified_%1pct.").arg(reductionPct) + inputExt;

    const QString outputPath = QFileDialog::getSaveFileName(
        this->widgetMain(),
        tr("Save simplified mesh as"),
        defaultOut,
        outFilter + tr(";;All Files (*)")
    );

    if (outputPath.isEmpty())
        return;

    // ── 5. Run on background thread ───────────────────────────────────────
    m_isRunning = true;

    QThread* thread = new QThread(this);
    QWidget* parentWgt = this->widgetMain();

    // Capture by value — everything the lambda needs
    const QString  inPath = inputPath;
    const QString  outPath = outputPath;
    const int      pct = reductionPct;
    const PMesh::EdgeCost cost = edgeCost;
    const int      origTriCount = origTris;

    connect(thread, &QThread::started, this, [=]() {

        // ── a) Load mesh ──────────────────────────────────────────────────
        QByteArray inBytes = inPath.toLocal8Bit();
        Mesh* mesh = new Mesh(inBytes.data());

        if (mesh->getNumTriangles() == 0) {
            delete mesh;
            QMetaObject::invokeMethod(qApp, [=]() {
                QMessageBox::critical(parentWgt, tr("Simplify Mesh"),
                    tr("Failed to load mesh on worker thread:\n%1").arg(inPath));
            }, Qt::QueuedConnection);
            QMetaObject::invokeMethod(thread, "quit", Qt::QueuedConnection);
            return;
        }

        // ── b) Build progressive mesh & collapse edges ────────────────────
        PMesh* pmesh = new PMesh(mesh, cost);

        // How many collapses do we need for the requested reduction?
        // Each collapse removes roughly 2 triangles on average.
        // Target: origTris * pct/100 triangles to remove.
        const int targetRemove = int(origTriCount * (pct / 100.0));
        const int maxCollapses = pmesh->numCollapses();
        // Each collapse in the list removes approximately 2 tris
        // Use numVisTris() to check live count
        int collapsesDone = 0;
        while (collapsesDone < maxCollapses) {
            const int visBefore = pmesh->numVisTris();
            if (!pmesh->collapseEdge()) break;
            collapsesDone++;
            const int removed = origTriCount - pmesh->numVisTris();
            if (removed >= targetRemove) break;
        }

        // ── c) Save result ────────────────────────────────────────────────
        const int finalTris = pmesh->numVisTris();

        // saveToFile works on the internal _newmesh which reflects
        // the current collapse state via active flags on triangles.
        // We call it through the Mesh pointer inside PMesh.
        QByteArray outBytes = outPath.toLocal8Bit();
        const bool saved = pmesh->getMesh()->saveToFile(outBytes.data());

        delete pmesh;
        delete mesh;

        // ── d) Report ─────────────────────────────────────────────────────
        QMetaObject::invokeMethod(qApp, [=]() {
            if (saved) {
                QMessageBox::information(parentWgt,
                    tr("Simplify Mesh"),
                    tr("Simplification complete!\n\n"
                       "Original triangles : %1\n"
                       "Remaining triangles: %2\n"
                       "Reduction          : %3%\n\n"
                       "Saved to:\n%4")
                        .arg(origTriCount)
                        .arg(finalTris)
                        .arg(pct)
                        .arg(outPath));
            }
 else {
  QMessageBox::critical(parentWgt,
      tr("Simplify Mesh"),
      tr("Simplification succeeded but failed to save file:\n%1").arg(outPath));
}
}, Qt::QueuedConnection);

QMetaObject::invokeMethod(thread, "quit", Qt::QueuedConnection);
});

connect(thread, &QThread::finished, this, [this]() {
    m_isRunning = false;
});
connect(thread, &QThread::finished, thread, &QObject::deleteLater);

thread->start();
}

} // namespace Mayo

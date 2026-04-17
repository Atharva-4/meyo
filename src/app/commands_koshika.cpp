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
#include <QtCore/QTimer>
#include "commands_file.h"
//simplification

#include <QtCore/QTimer>
#include <QtCore/QDir>
#include <QtCore/QStandardPaths>
#include <QtCore/QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>

#define _CRT_SECURE_NO_WARNINGS


#include "../gui/gui_document.h"

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <TopoDS_Compound.hxx>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtCore/QTimer>
#include <QtCore/QDir>
#include <QtCore/QStandardPaths>

#include "../qtcommon/filepath_conv.h"

// ── Your original mesh-simplification headers ────────────────────────────
#include "mesh.h"
#include "pmesh.h"

#include "StlHollowing.h"

//statistics
#include "MeshRepairStatistics.h"
#include <QtWidgets/QPushButton>
#include <QPushButton>

//water tight mesh
#include "WatertightMesh.h"
#include <QProgressDialog>

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
        guiDoc->graphicsView()->Redraw();

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
        action->setText(tr("Automatic Fill"));
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
        action->setText(tr("Mannual"));
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

        // Fixed method (best quality): Quadric
        constexpr PMesh::EdgeCost fixedEdgeCost = PMesh::QUADRIC;

        // Reduction slider
        auto* labelReduction = new QLabel(tr("Triangle reduction by:"), &dlg);


        auto* slider = new QSlider(Qt::Horizontal, &dlg);
        slider->setRange(1, 99);
        slider->setValue(50);

        auto* spinBox = new QSpinBox(&dlg);
        spinBox->setRange(1, 99);
        spinBox->setValue(50);
        spinBox->setSuffix(" %");

        auto* labelResult = new QLabel(&dlg);
        auto* labelPreviewStatus = new QLabel(&dlg);
        labelPreviewStatus->setStyleSheet(QStringLiteral("color:#666;"));
        labelPreviewStatus->setText(tr("Move the slider to update preview estimate"));
        auto* checkOpenPreview = new QCheckBox(tr("Generate simplified preview file while moving slider"), &dlg);
        checkOpenPreview->setChecked(false);

        auto* previewDebounce = new QTimer(&dlg);
        previewDebounce->setSingleShot(true);
        auto* previewWatcher = new QFutureWatcher<bool>(&dlg);
        bool previewBusy = false;
        bool previewQueued = false;
        int queuedPreviewPct = 0;

        const auto drawer = guiDoc->graphicsScene()->drawerDefault();
        const double baseDeviationCoeff = drawer->DeviationCoefficient();
        const double baseDeviationAngle = drawer->DeviationAngle();
        auto runSimplificationToFile = [&](const QString& outPath, int pct, PMesh::EdgeCost cost) {
            QByteArray inBytes = inputPath.toLocal8Bit();
            Mesh mesh(inBytes.data());
            if (mesh.getNumTriangles() == 0)
                return false;

            PMesh pmesh(&mesh, cost);
            const int targetRemove = int(origTris * (pct / 100.0));
            const int maxCollapses = pmesh.numCollapses();
            int collapsesDone = 0;
            while (collapsesDone < maxCollapses) {
                if (!pmesh.collapseEdge())
                    break;

                ++collapsesDone;
                const int removed = origTris - pmesh.numVisTris();
                if (removed >= targetRemove)
                    break;
            }

            QByteArray outBytes = outPath.toLocal8Bit();
            return pmesh.getMesh()->saveToFile(outBytes.data());
            };

        auto applyViewportPreviewFromReduction = [=](int reductionPct) {
            // This is a display-only preview (coarser tessellation), final mesh is still
            // computed by the simplification worker when user validates.
            const double ratio = std::clamp(reductionPct / 100.0, 0.0, 0.99);
            const double previewCoeff = baseDeviationCoeff * (1.0 + ratio * 8.0);
            const double previewAngle = std::min(baseDeviationAngle * (1.0 + ratio * 4.0), 1.2);

            drawer->SetDeviationCoefficient(previewCoeff);
            drawer->SetDeviationAngle(previewAngle);

            guiDoc->graphicsScene()->foreachDisplayedObject([&](const GraphicsObjectPtr& obj) {
                guiDoc->graphicsScene()->recomputeObjectPresentation(obj);
                });
            guiDoc->graphicsView().redraw();
            };

        QObject::connect(&dlg, &QDialog::finished, &dlg, [&](int) {
            drawer->SetDeviationCoefficient(baseDeviationCoeff);
            drawer->SetDeviationAngle(baseDeviationAngle);
            guiDoc->graphicsScene()->foreachDisplayedObject([&](const GraphicsObjectPtr& obj) {
                guiDoc->graphicsScene()->recomputeObjectPresentation(obj);
                });
            guiDoc->graphicsView().redraw();

            if (previewWatcher->isRunning()) {
                previewWatcher->cancel();
                previewWatcher->waitForFinished();
            }
            });

        QObject::connect(previewWatcher, &QFutureWatcher<bool>::finished, &dlg, [&]() {
            previewBusy = false;
            const bool okPreview = previewWatcher->result();
            labelPreviewStatus->setText(okPreview ? tr("Preview file updated") : tr("Preview generation failed"));

            if (previewQueued && checkOpenPreview->isChecked()) {
                previewQueued = false;
                const int pct = queuedPreviewPct;

                const QString tmpDir = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
                const QString previewPath = QDir(tmpDir).filePath(
                    QString("mayo_simplify_preview_%1.%2")
                    .arg(QFileInfo(inputPath).completeBaseName())
                    .arg(inputExt)
                );

                previewBusy = true;
                labelPreviewStatus->setText(tr("Generating preview..."));
                previewWatcher->setFuture(QtConcurrent::run([=]() {
                    return runSimplificationToFile(previewPath, pct, fixedEdgeCost);
                    }));
            }
            });


        auto updateResultLabel = [&]() {
            const int pct = spinBox->value();
            const int remaining = qMax(1, int(origTris * (1.0 - pct / 100.0)));
            labelResult->setText(
                tr("→  ~%1 triangles remaining  (%2% removed)")
                .arg(remaining).arg(pct));
            };
        updateResultLabel();

        QObject::connect(previewDebounce, &QTimer::timeout, &dlg, [&]() {
            updateResultLabel();
            const int pct = spinBox->value();
            applyViewportPreviewFromReduction(spinBox->value());

            if (checkOpenPreview->isChecked()) {
                if (previewBusy) {
                    previewQueued = true;
                    queuedPreviewPct = pct;
                    labelPreviewStatus->setText(tr("Preview queued..."));
                }
                else {
                    previewBusy = true;
                    labelPreviewStatus->setText(tr("Generating preview..."));
                    const QString tmpDir = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
                    const QString previewPath = QDir(tmpDir).filePath(
                        QString("mayo_simplify_preview_%1.%2")
                        .arg(QFileInfo(inputPath).completeBaseName())
                        .arg(inputExt)
                    );
                    previewWatcher->setFuture(QtConcurrent::run([=]() {
                        return runSimplificationToFile(previewPath, pct, fixedEdgeCost);
                        }));

                }
                return;
            }

            labelPreviewStatus->setText(tr("Preview updated"));
            });

        // Sync slider ↔ spinbox
        QObject::connect(slider, &QSlider::valueChanged,
            spinBox, &QSpinBox::setValue);
        QObject::connect(spinBox, qOverload<int>(&QSpinBox::valueChanged),
            slider, &QSlider::setValue);
        QObject::connect(spinBox, qOverload<int>(&QSpinBox::valueChanged),
            &dlg, [=](int) {
                labelPreviewStatus->setText(tr("Generating preview..."));
                previewDebounce->start(180);
            });

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
        layout->addWidget(labelReduction);
        layout->addLayout(sliderRow);
        layout->addWidget(labelResult);
        layout->addWidget(labelPreviewStatus);
        layout->addWidget(checkOpenPreview);
        layout->addSpacing(6);
        layout->addWidget(btnBox);

        if (dlg.exec() != QDialog::Accepted)
            return;

        const int reductionPct = spinBox->value();
        const PMesh::EdgeCost edgeCost = fixedEdgeCost;

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


    /////////////////////////////////////////////////////////////////////////////////
    // 
    // 
    // 
    ////////////////////////////////////////////////////////////////////////////////
    // ─────────────────────────────────────────────────────────────────────────────
    //  Constructor
    // ─────────────────────────────────────────────────────────────────────────────
    CommandHollowing::CommandHollowing(IAppContext* context)
        : Command(context)
    {
        auto* action = new QAction(this);
        action->setText(Command::tr("Hollow Mesh"));
        action->setToolTip(Command::tr(
            "Create a hollow version of the STL mesh by adding an inward-offset inner shell"));
        this->setAction(action);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  execute()
    // ─────────────────────────────────────────────────────────────────────────────
    void CommandHollowing::execute()
    {
        if (m_isRunning)
            return;

        // ── 1. Get the currently open document ───────────────────────────────────
        GuiDocument* guiDoc = this->currentGuiDocument();
        if (!guiDoc) {
            QMessageBox::warning(this->widgetMain(),
                tr("Hollow Mesh"),
                tr("No document is currently open.\n"
                    "Please open an STL file first."));
            return;
        }

        const FilePath filePath = guiDoc->document()->filePath();
        if (filePath.empty()) {
            QMessageBox::warning(this->widgetMain(),
                tr("Hollow Mesh"),
                tr("Current document has no file path associated with it."));
            return;
        }

        const QString inputPath = QString::fromStdString(filePath.u8string());
        const QString inputExt = QFileInfo(inputPath).suffix().toLower();
        const QString inputName = QFileInfo(inputPath).fileName();

        // Hollowing only supports STL
        if (inputExt != "stl") {
            QMessageBox::warning(this->widgetMain(),
                tr("Hollow Mesh"),
                tr("Unsupported file type: .%1\n"
                    "Only STL files can be hollowed.").arg(inputExt));
            return;
        }

        // ── 2. Load triangles now so we can show real triangle count in dialog ───
        std::vector<StlHollowing::Triangle> triangles;
        try {
            triangles = StlHollowing::loadStlFile(inputPath.toStdString());
        }
        catch (const std::exception& ex) {
            QMessageBox::critical(this->widgetMain(),
                tr("Hollow Mesh"),
                tr("Failed to load STL file:\n%1\n\nReason: %2")
                .arg(inputPath)
                .arg(QString::fromStdString(ex.what())));
            return;
        }

        if (triangles.empty()) {
            QMessageBox::critical(this->widgetMain(),
                tr("Hollow Mesh"),
                tr("STL file contains no triangles:\n%1\n\n"
                    "File may be empty or corrupt.").arg(inputPath));
            return;
        }

        const bool   isBinary = StlHollowing::isFileBinary(inputPath.toStdString());
        const int    origTris = static_cast<int>(triangles.size());

        // ── 3. Dialog ─────────────────────────────────────────────────────────────
        QDialog dlg(this->widgetMain());
        dlg.setWindowTitle(tr("Hollow Mesh"));
        dlg.setMinimumWidth(420);

        // Info label
        auto* labelFile = new QLabel(
            tr("<b>File:</b> %1<br><b>Original triangles:</b> %2")
            .arg(inputName)
            .arg(origTris),
            &dlg);
        labelFile->setWordWrap(true);

        auto* labelInnerOffset = new QLabel(tr("Inner offset:"), &dlg);
        auto* sliderInnerOffset = new QSlider(Qt::Horizontal, &dlg);
        sliderInnerOffset->setRange(1, 10000);
        sliderInnerOffset->setValue(500);
        auto* spinInnerOffset = new QDoubleSpinBox(&dlg);
        spinInnerOffset->setRange(0.001, 100.0);
        spinInnerOffset->setSingleStep(0.1);
        spinInnerOffset->setDecimals(3);
        spinInnerOffset->setValue(0.5);
        spinInnerOffset->setSuffix(" mm");

        auto* labelOuterOffset = new QLabel(tr("Outer offset:"), &dlg);
        auto* sliderOuterOffset = new QSlider(Qt::Horizontal, &dlg);
        sliderOuterOffset->setRange(0, 10000);
        sliderOuterOffset->setValue(0);
        auto* spinOuterOffset = new QDoubleSpinBox(&dlg);
        spinOuterOffset->setRange(0.0, 100.0);
        spinOuterOffset->setSingleStep(0.1);
        spinOuterOffset->setDecimals(3);
        spinOuterOffset->setValue(0.0);
        spinOuterOffset->setSuffix(" mm");

        auto* innerOffsetRow = new QHBoxLayout;
        innerOffsetRow->addWidget(labelInnerOffset);
        innerOffsetRow->addWidget(sliderInnerOffset);
        innerOffsetRow->addWidget(spinInnerOffset);

        auto* outerOffsetRow = new QHBoxLayout;
        outerOffsetRow->addWidget(labelOuterOffset);
        outerOffsetRow->addWidget(sliderOuterOffset);
        outerOffsetRow->addWidget(spinOuterOffset);

        const auto syncSliderFromSpin = [](QSlider* slider, double valueMm) {
            slider->setValue(qRound(valueMm * 1000.0));
            };
        const auto mmFromSlider = [](int sliderValue) {
            return sliderValue / 1000.0;
            };
        QObject::connect(sliderInnerOffset, &QSlider::valueChanged, &dlg, [=](int value) {
            QSignalBlocker guard(spinInnerOffset);
            spinInnerOffset->setValue(mmFromSlider(value));
            });
        QObject::connect(spinInnerOffset, qOverload<double>(&QDoubleSpinBox::valueChanged), &dlg, [=](double value) {
            QSignalBlocker guard(sliderInnerOffset);
            syncSliderFromSpin(sliderInnerOffset, value);
            });

        QObject::connect(sliderOuterOffset, &QSlider::valueChanged, &dlg, [=](int value) {
            QSignalBlocker guard(spinOuterOffset);
            spinOuterOffset->setValue(mmFromSlider(value));
            });
        QObject::connect(spinOuterOffset, qOverload<double>(&QDoubleSpinBox::valueChanged), &dlg, [=](double value) {
            QSignalBlocker guard(sliderOuterOffset);
            syncSliderFromSpin(sliderOuterOffset, value);
            });

        // Output triangle count label (updates live as user changes thickness)
        auto* labelResult = new QLabel(&dlg);
        auto updateResultLabel = [&]() {
            labelResult->setText(
                tr("→  Output triangles: ~%1  (outer shell + inner shell)\n"
                    "    Inner: %2 mm, Outer: %3 mm")
                .arg(origTris * 2)
                .arg(spinInnerOffset->value(), 0, 'f', 3)
                .arg(spinOuterOffset->value(), 0, 'f', 3));
            };
        updateResultLabel();


        QObject::connect(spinInnerOffset, qOverload<double>(&QDoubleSpinBox::valueChanged), &dlg, [&](double) {
            updateResultLabel();
            });
        QObject::connect(spinOuterOffset, qOverload<double>(&QDoubleSpinBox::valueChanged), &dlg, [&](double) {
            updateResultLabel();
            });

        // Info note
        auto* labelInfo = new QLabel(
            tr("The outer shell can be independently offset outward.\n"
                "The inner shell is offset inward and winding is reversed.\n"
                "Use the sliders to tune inner and outer offsets."),
            &dlg);
        labelInfo->setWordWrap(true);
        labelInfo->setStyleSheet(QStringLiteral("color:#666;"));

        auto* btnBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        QObject::connect(btnBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        QObject::connect(btnBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

        auto* layout = new QVBoxLayout(&dlg);
        layout->setSpacing(10);
        layout->setContentsMargins(16, 16, 16, 16);
        layout->addWidget(labelFile);
        layout->addSpacing(6);
        layout->addLayout(innerOffsetRow);
        layout->addLayout(outerOffsetRow);
        layout->addWidget(labelResult);
        layout->addWidget(labelInfo);
        layout->addSpacing(6);
        layout->addWidget(btnBox);

        if (dlg.exec() != QDialog::Accepted)
            return;

        const float innerOffset = static_cast<float>(spinInnerOffset->value());
        const float outerOffset = static_cast<float>(spinOuterOffset->value());

        // ── 4. Ask where to save ──────────────────────────────────────────────────
        const QString defaultOut =
            QFileInfo(inputPath).absolutePath() + "/" +
            QFileInfo(inputPath).completeBaseName() +
            QString("_hollow_i%1_o%2mm.stl")
            .arg(innerOffset, 0, 'f', 2)
            .arg(outerOffset, 0, 'f', 2);

        const QString outputPath = QFileDialog::getSaveFileName(
            this->widgetMain(),
            tr("Save hollow STL as"),
            defaultOut,
            tr("STL Files (*.stl);;All Files (*)")
        );

        if (outputPath.isEmpty())
            return;

        // ── 5. Run on background thread ───────────────────────────────────────────
        m_isRunning = true;

        QThread* thread = new QThread(this);
        QWidget* parentWgt = this->widgetMain();

        // Capture everything by value for the worker lambda
        const std::vector<StlHollowing::Triangle> trisCapture = std::move(triangles);
        const QString outPath = outputPath;
        const bool    binFmt = isBinary;
        const float   innerOff = innerOffset;
        const float   outerOff = outerOffset;
        const int     origTriCount = origTris;

        connect(thread, &QThread::started, this, [=]() {

            // ── a) Build hollow mesh ──────────────────────────────────────────────
            std::vector<StlHollowing::Triangle> hollowed =
                StlHollowing::buildHollowMeshWithOffsets(trisCapture, innerOff, outerOff);

            const int finalTris = static_cast<int>(hollowed.size());

            // ── b) Save result ────────────────────────────────────────────────────
            bool    saved = false;
            QString errorMsg;
            try {
                StlHollowing::saveStlFile(outPath.toStdString(), hollowed, binFmt);
                saved = true;
            }
            catch (const std::exception& ex) {
                errorMsg = QString::fromStdString(ex.what());
            }

            // ── c) Report ─────────────────────────────────────────────────────────
            QMetaObject::invokeMethod(qApp, [=]() {
                if (saved) {
                    QMessageBox::information(parentWgt,
                        tr("Hollow Mesh"),
                        tr("Hollowing complete!\n\n"
                            "Original triangles : %1\n"
                            "Output triangles   : %2  (outer + inner shells)\n"
                            "Inner offset       : %3 mm\n"
                            "Outer offset       : %4 mm\n\n"
                            "Saved to:\n%5")
                        .arg(origTriCount)
                        .arg(finalTris)
                        .arg(innerOff, 0, 'f', 3)
                        .arg(outerOff, 0, 'f', 3)
                        .arg(outPath));
                }
                else {
                    QMessageBox::critical(parentWgt,
                        tr("Hollow Mesh"),
                        tr("Hollowing succeeded but failed to save file:\n%1\n\nReason: %2")
                        .arg(outPath)
                        .arg(errorMsg));
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


    //////////////////////////////////////////////////////////////////////////
    //
    //
    //Statistic
    //
    ////////////////////////////////////////////////////////////////////////////
    

    CommandMeshRepairStatistics::CommandMeshRepairStatistics(IAppContext* context)
        : Command(context)
    {
        auto action = new QAction(this);
        action->setText(tr("Mesh Statistics"));   // ✅ FIX
        action->setToolTip(tr(
            "View mesh statistics like triangles, vertices, holes, "
            "non-manifold edges, and self-intersections."
        ));
        setAction(action);
    }

    void CommandMeshRepairStatistics::execute()
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
        QWidget* parent = this->widgetMain();

        std::string errorMessage;
        const auto stats = computeMeshRepairStatsFromMeshFile(inPath.toStdString(), &errorMessage);

        if (!stats) {
            QMessageBox::warning(parent, tr("Mesh Statistics"),
                tr("Could not compute statistics.\n%1").arg(QString::fromStdString(errorMessage)));
            return;
        }

        QDialog dlg(parent);
        dlg.setWindowTitle(tr("Mesh Statistics"));

        auto* layout = new QVBoxLayout(&dlg);

        auto* label = new QLabel(&dlg);
        label->setText(tr(
            "Format: %1\n"
            "Triangles: %2\n"
            "Unique Vertices: %3\n"
            "Duplicate Vertices: %4\n"
            "Holes: %5\n"
            "Non-manifold Edges: %6\n"
            "Self-intersections: %7"
        )
            .arg(QString::fromStdString(stats->inputFormat))
            .arg(stats->triangleCount)
            .arg(stats->vertexCount)
            .arg(stats->duplicateVertices)
            .arg(stats->holeCount)
            .arg(stats->nonManifoldEdges)
            .arg(stats->selfIntersectionPairs)
        );

        layout->addWidget(label);

        // Only Close button
        auto* btnClose = new QPushButton(tr("Close"), &dlg);
        layout->addWidget(btnClose);

        connect(btnClose, &QPushButton::clicked, &dlg, &QDialog::accept);

        dlg.exec();
    }



    //////////////////////////////////////////////////////////////////////////
    //
    //Auto Repair
    //
    ////////////////////////////////////////////////////////////////////////////
    CommandMeshAutoRepair::CommandMeshAutoRepair(IAppContext* context)
        : Command(context)
    {
        auto action = new QAction(this);
        action->setText(tr("Auto Repair"));
        action->setToolTip(tr(
            "Automatically repair the mesh (fill holes, fix non-manifold edges "
            "and remove self-intersections). Supports STL files."
        ));
        setAction(action);
    }

    void CommandMeshAutoRepair::execute()
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
        QWidget* parent = this->widgetMain();

        std::string errorMessage;
        const auto stats = computeMeshRepairStatsFromMeshFile(inPath.toStdString(), &errorMessage);
        if (!stats) {
            QMessageBox::warning(parent, tr("Mesh Statistics"),
                tr("Could not compute statistics.\n%1").arg(QString::fromStdString(errorMessage)));
            return;
        }

        QDialog dlg(parent);
        dlg.setWindowTitle(tr("Koshika Statistics"));
        auto* layout = new QVBoxLayout(&dlg);

        auto* label = new QLabel(&dlg);
        label->setText(tr("Format: %1\n"
            "Triangles: %2\n"
            "Unique Vertices: %3\n"
            "Duplicate Vertices: %4\n"
            "Holes: %5\n"
            "Non-manifold Edges: %6\n"
            "Self-intersections: %7\n"
        )
            .arg(QString::fromStdString(stats->inputFormat))
            .arg(stats->triangleCount)
            .arg(stats->vertexCount)
            .arg(stats->duplicateVertices)
            .arg(stats->holeCount)
            .arg(stats->nonManifoldEdges)
            .arg(stats->selfIntersectionPairs)
        );
        layout->addWidget(label);

        auto* btnAutoRepair = new QPushButton(tr("Auto Repair"), &dlg);
        auto* btnClose = new QPushButton(tr("Close"), &dlg);
        auto* rowButtons = new QHBoxLayout;
        rowButtons->addWidget(btnAutoRepair);
        rowButtons->addWidget(btnClose);
        layout->addLayout(rowButtons);

        connect(btnClose, &QPushButton::clicked, &dlg, &QDialog::reject);
        connect(btnAutoRepair, &QPushButton::clicked, &dlg, [=]() {
            if (m_isRunning)
                return;

            if (QFileInfo(inPath).suffix().compare("stl", Qt::CaseInsensitive) != 0) {
                QMessageBox::information(parent,
                    tr("Auto Repair"),
                    tr("Auto Repair currently supports STL files only. Statistics support STL/OBJ/PLY."));
                return;
            }

            const QString outPath = QFileDialog::getSaveFileName(
                parent,
                tr("Save repaired STL"),
                QFileInfo(inPath).completeBaseName() + "_repaired.stl",
                tr("STL Files (*.stl);;All Files (*)")
            );
            if (outPath.isEmpty())
                return;

            m_isRunning = true;
            QThread* thread = new QThread(this);
            auto* worker = new StlHoleFillingWorker();
            worker->moveToThread(thread);

            connect(thread, &QThread::started, this, [=]() {
                QMetaObject::invokeMethod(worker, "process",
                    Qt::QueuedConnection,
                    Q_ARG(QString, inPath),
                    Q_ARG(QString, outPath));
                });

            connect(worker, &StlHoleFillingWorker::finished, this, [=]() {
                QMessageBox::information(parent, tr("Auto Repair"),
                    tr("Repaired STL written to:\n%1").arg(outPath));
                m_isRunning = false;
                thread->quit();
                });

            connect(worker, &StlHoleFillingWorker::error, this, [=](const QString& msg) {
                QMessageBox::critical(parent, tr("Auto Repair Error"), msg);
                m_isRunning = false;
                thread->quit();
                });

            connect(thread, &QThread::finished, worker, &QObject::deleteLater);
            connect(thread, &QThread::finished, thread, &QObject::deleteLater);
            thread->start();
            });

        dlg.exec();
    }

    //////////////////////////////////////////////////////////////////////////
   //
   //  Watertight Mesh
   //
   //////////////////////////////////////////////////////////////////////////

    CommandWatertightMesh::CommandWatertightMesh(IAppContext* context)
        : Command(context)
    {
        auto action = new QAction(this);
        action->setText(tr("Make Watertight"));
        action->setToolTip(tr("Run a two-stage voxel repair to make the mesh "
            "fully watertight (closes holes, removes "
            "self-intersections & non-manifold geometry)"));
        setAction(action);
    }


    void CommandWatertightMesh::execute()
    {
        if (m_isRunning)
            return;

        // ── 1. Guard: need an open document ──────────────────────────────────
        GuiDocument* guiDoc = this->currentGuiDocument();
        if (!guiDoc) {
            QMessageBox::warning(this->widgetMain(),
                tr("Watertight Mesh"),
                tr("Please open a mesh file first."));
            return;
        }

        const FilePath filePath = guiDoc->document()->filePath();
        if (filePath.empty()) {
            QMessageBox::warning(this->widgetMain(),
                tr("Watertight Mesh"),
                tr("The current document has no file path."));
            return;
        }

        const QString inPath =
            QString::fromStdString(filePath.u8string());

        // ── 2. Check format (STL / OBJ / PLY supported) ──────────────────────
        const QString ext = QFileInfo(inPath).suffix().toLower();
        if (ext != "stl" && ext != "obj" && ext != "ply") {
            QMessageBox::information(this->widgetMain(),
                tr("Watertight Mesh"),
                tr("Watertight repair supports STL, OBJ and PLY files.\n"
                    "Current file: %1").arg(inPath));
            return;
        }

        //// ── 3. Voxel resolution dialog ────────────────────────────────────────
        //QDialog optDlg(this->widgetMain());
        //optDlg.setWindowTitle(tr("Watertight Repair Options"));

        //auto* vLayout = new QVBoxLayout(&optDlg);

        //// Resolution spin
        //auto* resLabel = new QLabel(
        //    tr("Voxel resolution  (higher = more detail, slower):"), &optDlg);
        //auto* resSpin = new QSpinBox(&optDlg);
        //resSpin->setRange(32, 512);
        //resSpin->setValue(128);
        //resSpin->setSingleStep(32);
        //resSpin->setSuffix(tr("  voxels"));

        //// Info label
        //auto* infoLabel = new QLabel(
        //    tr("<small>Recommended: 128 for most meshes, "
        //        "256 for high-detail models, "
        //        "512 for ultra-precision (uses ~1 GB RAM).</small>"),
        //    &optDlg);
        //infoLabel->setWordWrap(true);

        //auto* btnBox = new QDialogButtonBox(
        //    QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &optDlg);
        //connect(btnBox, &QDialogButtonBox::accepted, &optDlg, &QDialog::accept);
        //connect(btnBox, &QDialogButtonBox::rejected, &optDlg, &QDialog::reject);

        //vLayout->addWidget(resLabel);
        //vLayout->addWidget(resSpin);
        //vLayout->addWidget(infoLabel);
        //vLayout->addWidget(btnBox);

        //if (optDlg.exec() != QDialog::Accepted)
        //    return;

        //const int voxelRes = resSpin->value();
        const int voxelRes = 128;

        // ── 4. Output file path ───────────────────────────────────────────────
        const QString defaultOut =
            QFileInfo(inPath).absolutePath() + '/' +
            QFileInfo(inPath).completeBaseName() +
            QString("_watertight_v%1.%2").arg(voxelRes).arg(ext);

        const QString outPath = QFileDialog::getSaveFileName(
            this->widgetMain(),
            tr("Save watertight mesh as"),
            defaultOut,
            tr("STL Files (*.stl);;OBJ Files (*.obj);;All Files (*)")
        );
        if (outPath.isEmpty())
            return;

        // ── 5. Progress dialog ────────────────────────────────────────────────
        auto* progressDlg = new QProgressDialog(
            tr("Preparing watertight repair…"),
            QString() /*no cancel button*/,
            0, 100,
            this->widgetMain());
        progressDlg->setWindowTitle(tr("Watertight Mesh"));
        progressDlg->setWindowModality(Qt::WindowModal);
        progressDlg->setMinimumDuration(0);
        progressDlg->setValue(0);

        // ── 6. Run on background thread ───────────────────────────────────────
        m_isRunning = true;

        QWidget* parentWgt = this->widgetMain();
        QString  inPathCopy = inPath;
        QString  outPathCopy = outPath;

        // We use QFutureWatcher so the progress dialog pumps the event loop
        auto* watcher = new QFutureWatcher<QString>(this);

        QObject::connect(watcher, &QFutureWatcher<QString>::progressValueChanged,
            progressDlg, &QProgressDialog::setValue);

        QObject::connect(watcher, &QFutureWatcher<QString>::finished,
            this, [this, watcher, progressDlg,
            parentWgt, outPathCopy]()
            {
                progressDlg->close();
                progressDlg->deleteLater();

                const QString errMsg = watcher->result();
                watcher->deleteLater();

                m_isRunning = false;

                if (!errMsg.isEmpty()) {
                    QMessageBox::critical(parentWgt,
                        tr("Watertight Mesh"),
                        tr("Repair failed:\n%1").arg(errMsg));
                    return;
                }

                QMessageBox::information(parentWgt,
                    tr("Watertight Mesh"),
                    tr("Watertight repair complete!\n\n"
                        "The repaired mesh has been saved to:\n%1")
                    .arg(outPathCopy));
            });

        // Run the algorithm concurrently

        QFuture<QString> future = QtConcurrent::run(
            [inPathCopy, outPathCopy, voxelRes, progressDlg]() -> QString
            {
                // progress callback – posts value back to the progress dialog in the GUI thread
                Mayo::WTProgressCallback cb =
                    [progressDlg](int pct, const std::string&) {
                    QMetaObject::invokeMethod(progressDlg, "setValue", Qt::QueuedConnection, Q_ARG(int, pct));
                    };

                const std::string err = Mayo::wtRepairMesh(
                    inPathCopy.toStdString(),
                    outPathCopy.toStdString(),
                    voxelRes,
                    cb
                );

                return QString::fromStdString(err);
            });

        watcher->setFuture(future);
    }

} // namespace Mayo

#include "main_window.h"
#include "point_cloud_generator.h"
#include "ransac_detector.h"
#include <QDateTime>
#include <QMessageBox>
#include <QDir>
#include <QSplitter>
#include <QMenuBar>
#include <QMenu>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QCoreApplication>
#include <QStatusBar>
#include <QScrollBar>
#include <QFrame>
#include <QFileInfo>
#include <QSignalBlocker>
#include <QtConcurrent>
#include <array>
#include <functional>

namespace {

constexpr int kRoleSection = -2;
constexpr int kRoleCategory = -3;
constexpr int kRoleRemaining = -1;

struct ShapeCategory {
    DetectedShape::Type type;
    const char* name;
};

constexpr ShapeCategory kShapeCategories[] = {
    {DetectedShape::PLANE,        u8"平面"},
    {DetectedShape::CYLINDER,     u8"圆柱体"},
    {DetectedShape::SPHERE,       u8"球体"},
    {DetectedShape::CONE,         u8"圆锥体"},
    {DetectedShape::TORUS,        u8"圆环"},
    {DetectedShape::CIRCLE3D,     u8"三维圆"},
    {DetectedShape::ELLIPSE3D,    u8"三维椭圆"},
    {DetectedShape::NORMAL_PLANE, u8"法向平面"},
    {DetectedShape::PERP_PLANE,   u8"正交平面"},
    {DetectedShape::LINE,         u8"直线"},
    {DetectedShape::UNKNOWN,      u8"未知"},
};

void foreachShapeTreeItem(QTreeWidgetItem* node,
                          const std::function<void(QTreeWidgetItem*, int shapeIdx)>& visitor)
{
    if (!node) {
        return;
    }
    const int role = node->data(0, Qt::UserRole).toInt();
    if (role >= 0) {
        visitor(node, role);
        return;
    }
    for (int i = 0; i < node->childCount(); ++i) {
        foreachShapeTreeItem(node->child(i), visitor);
    }
}

QString shapeTypeBreakdown(const std::vector<DetectedShape>& shapes)
{
    std::array<int, DetectedShape::TYPE_COUNT> counts{};
    for (const auto& shape : shapes) {
        const int ti = static_cast<int>(shape.type);
        if (ti >= 0 && ti < static_cast<int>(DetectedShape::TYPE_COUNT)) {
            counts[static_cast<size_t>(ti)]++;
        }
    }

    QStringList parts;
    for (const auto& cat : kShapeCategories) {
        const int c = counts[static_cast<size_t>(cat.type)];
        if (c > 0) {
            parts << QString(u8"%1 %2").arg(QString::fromUtf8(cat.name)).arg(c);
        }
    }
    return parts.join(u8"，");
}

void saveDetectionResultsToDir(const QString& dir,
                               const QString& baseName,
                               const PointCloudT::Ptr& inputCloud,
                               const std::vector<DetectedShape>& shapes,
                               const PointCloudT::Ptr& remaining)
{
    QDir().mkpath(dir);
    if (inputCloud && !inputCloud->points.empty()) {
        const QString inputPath = dir + "/" + baseName + "_input.pcd";
        PointCloudGenerator::saveToPCD(inputCloud, inputPath.toStdWString());
    }
    for (size_t i = 0; i < shapes.size(); ++i) {
        const QString path = dir + "/" + baseName
            + QString("_shape_%1_%2.pcd")
                  .arg(i + 1)
                  .arg(QString::fromStdString(shapes[i].getTypeNameEN()));
        PointCloudGenerator::saveToPCD(shapes[i].inliers, path.toStdWString());
    }
    if (remaining && !remaining->points.empty()) {
        const QString remPath = dir + "/" + baseName + "_remaining_unclassified.pcd";
        PointCloudGenerator::saveToPCD(remaining, remPath.toStdWString());
    }
}

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , _currentCloud(new PointCloudT)
    , _remainingPoints(new PointCloudT)
    , _detectWatcher(new QFutureWatcher<DetectionResult>(this))
    , _loadWatcher(new QFutureWatcher<LoadResult>(this))
    , _generateWatcher(new QFutureWatcher<PointCloudT::Ptr>(this))
    , _saveWatcher(new QFutureWatcher<void>(this))
{
    setWindowTitle(u8"RANSAC 点云几何体检测");
    setMinimumSize(1280, 820);
    resize(1440, 900);

    setupUI();
    setupMenus();

    connect(_detectWatcher, &QFutureWatcher<DetectionResult>::finished,
            this, &MainWindow::onDetectionFinished);
    connect(_loadWatcher, &QFutureWatcher<LoadResult>::finished,
            this, &MainWindow::onLoadFinished);

    _loadProgressTimer = new QTimer(this);
    _loadProgressTimer->setInterval(1500);
    connect(_loadProgressTimer, &QTimer::timeout, this, &MainWindow::onLoadProgressTick);

    connect(_generateWatcher, &QFutureWatcher<PointCloudT::Ptr>::finished,
            this, &MainWindow::onGenerateFinished);
    connect(_saveWatcher, &QFutureWatcher<void>::finished,
            this, &MainWindow::onSaveFinished);

    QSettings settings;
    _lastSaveDir = settings.value("lastSaveDir", "").toString();
    _lastLoadDir = settings.value("lastLoadDir", "").toString();

    updateStatus(u8"就绪");
}

MainWindow::~MainWindow() = default;

void MainWindow::setupUI()
{
    setStyleSheet(R"(
        QMainWindow { background: #f0f4f8; }
        QMenuBar { background: #ffffff; color: #1e293b; border-bottom: 1px solid #e2e8f0; }
        QMenuBar::item { padding: 6px 14px; }
        QMenuBar::item:selected { background: #e0f2fe; }
        QMenu { background: #ffffff; border: 1px solid #e2e8f0; }
        QMenu::item { padding: 6px 20px; }
        QMenu::item:selected { background: #e0f2fe; }
        QGroupBox {
            font-size: 12px; font-weight: 600; color: #475569;
            border: 1px solid #e2e8f0; border-radius: 6px;
            margin-top: 14px; padding: 12px 8px 8px 8px; background: #ffffff;
        }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
        QPushButton {
            background: #ffffff; color: #334155; border: 1px solid #cbd5e1;
            border-radius: 5px; padding: 8px 12px; font-size: 12px;
        }
        QPushButton:hover { background: #f8fafc; border-color: #94a3b8; }
        QPushButton:disabled { color: #94a3b8; background: #f1f5f9; }
        QPushButton#primaryButton {
            background: #2563eb; color: #ffffff; border: none; font-weight: 600;
        }
        QPushButton#primaryButton:hover { background: #1d4ed8; }
        QPushButton#secondaryButton {
            background: #f1f5f9; color: #475569; border: 1px solid #e2e8f0;
            padding: 6px 10px; font-size: 11px;
        }
        QLabel { color: #64748b; font-size: 12px; }
        QLabel#titleLabel { font-size: 15px; font-weight: 700; color: #0f172a; }
        QTreeWidget {
            background: #ffffff; border: 1px solid #e2e8f0; border-radius: 5px;
            font-size: 12px; outline: none;
        }
        QTreeWidget::item { padding: 5px 4px; }
        QTreeWidget::item:selected { background: #dbeafe; color: #1e40af; }
        QTreeWidget::branch { background: #ffffff; }
        QTextEdit {
            background: #f8fafc; color: #64748b; border: 1px solid #e2e8f0;
            border-radius: 5px; padding: 6px; font-family: Consolas, monospace; font-size: 11px;
        }
        QStatusBar { background: #ffffff; color: #64748b; border-top: 1px solid #e2e8f0; }
        QSplitter::handle { background: #e2e8f0; width: 2px; }
        QFrame#sidePanel { background: #ffffff; border-right: 1px solid #e2e8f0; }
    )");

    QWidget* central = new QWidget;
    QHBoxLayout* rootLayout = new QHBoxLayout(central);
    rootLayout->setContentsMargins(0, 0, 0, 0);
    rootLayout->setSpacing(0);

    QFrame* sidePanel = new QFrame;
    sidePanel->setObjectName("sidePanel");
    sidePanel->setFixedWidth(340);
    QVBoxLayout* sideLayout = new QVBoxLayout(sidePanel);
    sideLayout->setContentsMargins(14, 14, 14, 14);
    sideLayout->setSpacing(10);

    QLabel* titleLabel = new QLabel(u8"RANSAC 检测工具");
    titleLabel->setObjectName("titleLabel");
    sideLayout->addWidget(titleLabel);

    _infoLabel = new QLabel;
    _infoLabel->setWordWrap(true);
    sideLayout->addWidget(_infoLabel);

    QGroupBox* fileGroup = new QGroupBox(u8"点云操作");
    QVBoxLayout* fileLayout = new QVBoxLayout(fileGroup);
    _generateBtn = new QPushButton(u8"生成合成点云");
    _generateBtn->setObjectName("primaryButton");
    connect(_generateBtn, &QPushButton::clicked, this, &MainWindow::generatePointCloud);
    QHBoxLayout* fileRow = new QHBoxLayout;
    _loadBtn = new QPushButton(u8"加载文件");
    _saveBtn = new QPushButton(u8"保存");
    connect(_loadBtn, &QPushButton::clicked, this, &MainWindow::loadPointCloud);
    connect(_saveBtn, &QPushButton::clicked, this, &MainWindow::savePointCloud);
    fileRow->addWidget(_loadBtn);
    fileRow->addWidget(_saveBtn);
    fileLayout->addWidget(_generateBtn);
    fileLayout->addLayout(fileRow);
    sideLayout->addWidget(fileGroup);

    QGroupBox* detectGroup = new QGroupBox(u8"检测");
    QVBoxLayout* detectLayout = new QVBoxLayout(detectGroup);
    _detectPresetCombo = new QComboBox;
    _detectPresetCombo->addItem(u8"游乐场 / 大场景（仅平面，推荐）");
    _detectPresetCombo->addItem(u8"机械零件（全几何体）");
    _detectPresetCombo->setToolTip(
        u8"游乐场点云：识别地面、墙面、平台等大平面；\n"
        u8"摩天轮钢架、三角网格杆件等会归入「未分类」（算法不支持三角形/杆件）。");
    detectLayout->addWidget(_detectPresetCombo);
    _detectBtn = new QPushButton(u8"开始检测");
    _detectBtn->setObjectName("primaryButton");
    connect(_detectBtn, &QPushButton::clicked, this, &MainWindow::detectShapes);
    QHBoxLayout* detectRow = new QHBoxLayout;
    _clearBtn = new QPushButton(u8"清除");
    _exportBtn = new QPushButton(u8"导出");
    connect(_clearBtn, &QPushButton::clicked, this, &MainWindow::clearAll);
    connect(_exportBtn, &QPushButton::clicked, this, &MainWindow::exportResults);
    detectRow->addWidget(_clearBtn);
    detectRow->addWidget(_exportBtn);
    detectLayout->addWidget(_detectBtn);
    detectLayout->addLayout(detectRow);
    sideLayout->addWidget(detectGroup);

    QGroupBox* resultGroup = new QGroupBox(u8"识别结果");
    QVBoxLayout* resultLayout = new QVBoxLayout(resultGroup);
    _resultsTree = new QTreeWidget;
    _resultsTree->setHeaderHidden(true);
    _resultsTree->setMinimumHeight(200);
    _resultsTree->setIndentation(16);
    connect(_resultsTree, &QTreeWidget::currentItemChanged,
            this, [this](QTreeWidgetItem* item) { onTreeItemSelected(item); });
    connect(_resultsTree, &QTreeWidget::itemChanged,
            this, &MainWindow::onTreeItemChanged);
    resultLayout->addWidget(_resultsTree);

    QHBoxLayout* viewRow = new QHBoxLayout;
    QPushButton* showAllBtn = new QPushButton(u8"全选");
    showAllBtn->setObjectName("secondaryButton");
    QPushButton* deselectAllBtn = new QPushButton(u8"取消全选");
    deselectAllBtn->setObjectName("secondaryButton");
    connect(showAllBtn, &QPushButton::clicked, this, &MainWindow::showAllShapes);
    connect(deselectAllBtn, &QPushButton::clicked, this, &MainWindow::deselectAllShapes);
    viewRow->addWidget(showAllBtn);
    viewRow->addWidget(deselectAllBtn);
    resultLayout->addLayout(viewRow);

    _detailLabel = new QLabel;
    _detailLabel->setWordWrap(true);
    _detailLabel->setStyleSheet("color: #475569; font-size: 11px; padding: 4px;");
    resultLayout->addWidget(_detailLabel);
    sideLayout->addWidget(resultGroup, 2);

    QGroupBox* logGroup = new QGroupBox(u8"日志");
    QVBoxLayout* logLayout = new QVBoxLayout(logGroup);
    _logText = new QTextEdit;
    _logText->setReadOnly(true);
    _logText->setMaximumHeight(110);
    logLayout->addWidget(_logText);
    sideLayout->addWidget(logGroup, 1);

    _viewerWidget = new PointCloudViewerWidget;

    rootLayout->addWidget(sidePanel);
    rootLayout->addWidget(_viewerWidget, 1);

    setCentralWidget(central);
}

void MainWindow::setupMenus()
{
    QMenu* fileMenu = menuBar()->addMenu(u8"文件");
    QAction* loadAct = fileMenu->addAction(u8"加载点云");
    connect(loadAct, &QAction::triggered, this, &MainWindow::loadPointCloud);
    QAction* saveAct = fileMenu->addAction(u8"保存点云");
    connect(saveAct, &QAction::triggered, this, &MainWindow::savePointCloud);
    fileMenu->addSeparator();
    connect(fileMenu->addAction(u8"退出"), &QAction::triggered, this, &QMainWindow::close);

    QMenu* helpMenu = menuBar()->addMenu(u8"帮助");
    connect(helpMenu->addAction(u8"关于"), &QAction::triggered, [this]() {
        QMessageBox::about(this, u8"关于",
            u8"RANSAC 点云几何体检测\n\n"
            u8"算法: Schnabel et al. 2007 (qRANSAC_SD)\n"
            u8"支持: PCD / PLY / TXT / STL / OBJ");
    });
}

void MainWindow::logMessage(const QString& message)
{
    QString time = QDateTime::currentDateTime().toString("[HH:mm:ss]");
    _logText->append(QString("%1 %2").arg(time, message));
    _logText->verticalScrollBar()->setValue(_logText->verticalScrollBar()->maximum());
}

void MainWindow::logSuccess(const QString& message) { logMessage(message); }
void MainWindow::logWarning(const QString& message) { logMessage(message); }
void MainWindow::logError(const QString& message) { logMessage(message); }

void MainWindow::updateStatus(const QString& status)
{
    statusBar()->showMessage(status);
}

void MainWindow::setBusy(bool busy, const QString& message)
{
    _busy = busy;
    const bool enabled = !busy;
    if (_generateBtn) _generateBtn->setEnabled(enabled);
    if (_loadBtn) _loadBtn->setEnabled(enabled);
    if (_saveBtn) _saveBtn->setEnabled(enabled);
    if (_detectBtn) _detectBtn->setEnabled(enabled);
    if (_detectPresetCombo) _detectPresetCombo->setEnabled(enabled);
    if (_clearBtn) _clearBtn->setEnabled(enabled);
    if (_exportBtn) _exportBtn->setEnabled(enabled);

    if (_viewerWidget) {
        _viewerWidget->setTaskActive(busy, message);
    }

    if (busy) {
        setCursor(Qt::BusyCursor);
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
    } else {
        unsetCursor();
        updateStatus(u8"就绪");
    }
}

void MainWindow::generatePointCloud()
{
    if (_busy) return;

    logMessage(u8"正在生成合成点云...");
    setBusy(true, u8"生成点云…");

    auto future = QtConcurrent::run([]() {
        return PointCloudGenerator::generateSyntheticCloud();
    });
    _generateWatcher->setFuture(future);
}

void MainWindow::onGenerateFinished()
{
    PointCloudT::Ptr cloud = _generateWatcher->result();

    if (!cloud || cloud->points.empty()) {
        setBusy(false);
        logError(u8"生成失败");
        return;
    }

    _currentCloud = cloud;
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    _sourceBaseName = QString("generated_%1").arg(timestamp);

    _viewerWidget->setTaskMessage(u8"更新视图…");
    _resultsTree->clear();
    _detailLabel->clear();
    _viewerWidget->setPointCloud(_currentCloud);
    setBusy(false);

    _infoLabel->setText(QString(u8"已加载: %1 个点").arg(_currentCloud->points.size()));
    _infoLabel->setStyleSheet("color: #2563eb; font-weight: 500;");
    logSuccess(QString(u8"生成完成: %1 个点").arg(_currentCloud->points.size()));
}

void MainWindow::savePointCloud()
{
    if (_busy) return;
    if (!_currentCloud || _currentCloud->points.empty()) {
        logWarning(u8"没有可保存的点云");
        return;
    }
    QString dir = _lastSaveDir.isEmpty() ? QDir::currentPath() : _lastSaveDir;
    _viewerWidget->pauseRendering();
    QString fileName = QFileDialog::getSaveFileName(this, u8"保存点云", dir,
        u8"PCD 文件 (*.pcd);;PLY 文件 (*.ply);;所有文件 (*)");
    _viewerWidget->resumeRendering();
    if (fileName.isEmpty()) return;

    _lastSaveDir = QFileInfo(fileName).absolutePath();
    QSettings().setValue("lastSaveDir", _lastSaveDir);
    PointCloudGenerator::saveToPCD(_currentCloud, fileName.toStdWString());
    logSuccess(QString(u8"已保存: %1").arg(fileName));
}

void MainWindow::loadPointCloud()
{
    if (_busy) return;

    QString dir = _lastLoadDir.isEmpty() ? QDir::currentPath() : _lastLoadDir;
    _viewerWidget->pauseRendering();
    QString fileName = QFileDialog::getOpenFileName(this, u8"加载点云", dir,
        u8"点云文件 (*.pcd *.ply *.txt *.stl *.obj);;"
        u8"PCD (*.pcd);;PLY (*.ply);;TXT (*.txt);;STL (*.stl);;OBJ (*.obj);;所有文件 (*)");
    _viewerWidget->resumeRendering();
    if (fileName.isEmpty()) return;

    _lastLoadDir = QFileInfo(fileName).absolutePath();
    QSettings().setValue("lastLoadDir", _lastLoadDir);
    _sourceBaseName = QFileInfo(fileName).completeBaseName();

    logMessage(QString(u8"正在加载: %1").arg(fileName));
    setBusy(true, u8"读取点云…");
    if (_loadProgressTimer) {
        _loadProgressTimer->start();
    }

    const std::wstring path = fileName.toStdWString();
    auto future = QtConcurrent::run([path]() -> LoadResult {
        LoadResult result;
        auto raw = PointCloudGenerator::loadFromFile(path);
        if (!raw || raw->points.empty()) {
            return result;
        }
        const bool skipDedup = PointCloudGenerator::lastLoadMeta().txtStreamDownsampled;
        result.cloud = PointCloudGenerator::preprocess(raw, !skipDedup, 0.0f);
        result.streamDownsampled = PointCloudGenerator::lastLoadMeta().txtStreamDownsampled;
        result.sourceBytes = PointCloudGenerator::lastLoadMeta().fileSizeBytes;
        if (result.cloud && result.cloud->points.size() > 900000) {
            result.cloud = PointCloudGenerator::limitPointsForDisplay(
                result.cloud, 900000);
        }
        result.success = result.cloud && !result.cloud->points.empty();
        return result;
    });
    _loadWatcher->setFuture(future);
}

void MainWindow::onLoadProgressTick()
{
    if (!_busy || !_loadWatcher || !_loadWatcher->isRunning()) {
        return;
    }
    const PointCloudLoadMeta meta = PointCloudGenerator::lastLoadMeta();
    if (meta.fileSizeBytes == 0) {
        return;
    }
    const double pct = meta.txtLoadProgressPercent;
    const double mbDone = meta.txtBytesScanned / (1024.0 * 1024.0);
    const double mbTotal = meta.fileSizeBytes / (1024.0 * 1024.0);
    QString phase;
    if (meta.txtLoadPhase == 0) {
        phase = u8"扫描";
    } else if (meta.txtLoadPhase == 1) {
        phase = u8"生成点云";
    } else {
        phase = u8"上传视图";
    }
    const QString msg = QString(u8"读取点云… %1% (%2 / %3 MB) · %4")
        .arg(pct, 0, 'f', 0)
        .arg(mbDone, 0, 'f', 0)
        .arg(mbTotal, 0, 'f', 0)
        .arg(phase);
    if (_viewerWidget) {
        _viewerWidget->setTaskMessage(msg);
    }
    updateStatus(msg);
}

void MainWindow::onLoadFinished()
{
    if (_loadProgressTimer) {
        _loadProgressTimer->stop();
    }

    const LoadResult result = _loadWatcher->result();

    if (!result.success) {
        setBusy(false);
        logError(u8"加载失败（无法打开、格式无法识别或有效点数为 0；带引号 CSV 需使用新版程序）");
        updateStatus(u8"加载失败");
        return;
    }

    _currentCloud = result.cloud;
    _viewerWidget->setTaskMessage(u8"上传视图…");
    _resultsTree->clear();
    _detailLabel->clear();
    _viewerWidget->setPointCloud(_currentCloud);
    setBusy(false);

    _infoLabel->setText(QString(u8"已加载: %1 个点").arg(_currentCloud->points.size()));
    _infoLabel->setStyleSheet("color: #2563eb; font-weight: 500;");
    logSuccess(QString(u8"加载完成: %1 个点").arg(_currentCloud->points.size()));
    if (result.streamDownsampled && result.sourceBytes > 0) {
        logMessage(QString(u8"大 TXT 流式加载（原文件约 %1 MB → 目标约 90 万点，实际 %2 点）")
            .arg(result.sourceBytes / (1024.0 * 1024.0), 0, 'f', 1)
            .arg(_currentCloud->points.size()));
    }
}

void MainWindow::detectShapes()
{
    if (!_currentCloud || _currentCloud->points.empty()) {
        logWarning(u8"请先生成或加载点云");
        return;
    }
    if (_busy) return;

    setBusy(true, u8"检测中…");
    logMessage(u8"开始检测");
    _detectTimer.start();

    PointCloudT::Ptr cloud = _currentCloud;
    const int presetIndex = _detectPresetCombo ? _detectPresetCombo->currentIndex() : 0;
    auto future = QtConcurrent::run([cloud, presetIndex]() -> DetectionResult {
        RansacDetector::Config cfg;
        cfg.preset = (presetIndex == 0)
            ? RansacDetector::Config::Preset::Architecture
            : RansacDetector::Config::Preset::Full;
        RansacDetector detector(cfg);
        DetectionResult result;
        result.shapes = detector.detect(cloud);
        result.remaining = detector.getRemainingPoints();
        result.stats = detector.lastDetectStats();
        return result;
    });
    _detectWatcher->setFuture(future);
}

void MainWindow::onDetectionFinished()
{
    const DetectionResult result = _detectWatcher->result();

    _detectedShapes = result.shapes;
    _remainingPoints = result.remaining;
    _lastDetectStats = result.stats;

    _viewerWidget->setTaskMessage(u8"更新视图…");
    populateResultsTree();
    _viewerWidget->showDetectionResults(_detectedShapes, _remainingPoints, _currentCloud);

    logSuccess(QString(u8"检测完成: %1 个几何体 (%2)，耗时 %3 秒")
        .arg(_detectedShapes.size())
        .arg(shapeTypeBreakdown(_detectedShapes))
        .arg(_detectTimer.elapsed() / 1000.0, 0, 'f', 1));
    if (_detectPresetCombo && _detectPresetCombo->currentIndex() == 0) {
        logMessage(u8"游乐场模式: 仅提取大平面（地面/墙面/平台）；钢架与网格结构保留在未分类");
    }
    if (_lastDetectStats.detectPoints > 0) {
        const size_t accounted = _lastDetectStats.inlierSum + _lastDetectStats.remainingPoints;
        logMessage(QString(u8"检测统计: 显示点云 %1 → 实际检测 %2 点；形状内点 %3 + 未分类 %4 = %5")
            .arg(_lastDetectStats.loadedPoints)
            .arg(_lastDetectStats.detectPoints)
            .arg(_lastDetectStats.inlierSum)
            .arg(_lastDetectStats.remainingPoints)
            .arg(accounted));
        if (_lastDetectStats.loadedPoints > _lastDetectStats.detectPoints) {
            logMessage(QString(u8"说明: 列表中每个几何体的「内点」是检测子集上的统计，"
                u8"并非显示点云 %1 点的逐点归属；视图底图仍为完整显示点云。")
                .arg(_lastDetectStats.loadedPoints));
        }
    }

    _resultRunId = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    const QString dir = resultOutputDir();
    _lastResultDir = dir;

    const QString baseName = _sourceBaseName;
    PointCloudT::Ptr inputCloud = _currentCloud;
    std::vector<DetectedShape> shapes = _detectedShapes;
    PointCloudT::Ptr remaining = _remainingPoints;

    _viewerWidget->setTaskMessage(u8"保存结果…");
    auto future = QtConcurrent::run([dir, baseName, inputCloud, shapes, remaining]() {
        saveDetectionResultsToDir(dir, baseName, inputCloud, shapes, remaining);
    });
    _saveWatcher->setFuture(future);
}

void MainWindow::onSaveFinished()
{
    setBusy(false);
    logMessage(QString(u8"结果已保存: data/saved/%1/detect_%2/")
        .arg(_sourceBaseName, _resultRunId));
}

QString MainWindow::shapeFileBaseName(size_t index) const
{
    return QString("%1_shape_%2_%3")
        .arg(_sourceBaseName)
        .arg(index + 1)
        .arg(QString::fromStdString(_detectedShapes[index].getTypeNameEN()));
}

QString MainWindow::resultOutputDir() const
{
    // data/saved/{源文件名}/detect_{时间戳}/ — 同一文件多次检测互不覆盖
    return QCoreApplication::applicationDirPath()
        + "/data/saved/" + _sourceBaseName + "/detect_" + _resultRunId;
}

void MainWindow::populateResultsTree()
{
    _resultsTree->clear();

    if (_lastDetectStats.detectPoints > 0) {
        const size_t accounted = _lastDetectStats.inlierSum + _lastDetectStats.remainingPoints;
        auto* summary = new QTreeWidgetItem(_resultsTree);
        summary->setText(0, QString(u8"检测输入 %1 点（显示 %2 点）· 内点 %3 + 未分类 %4 = %5")
            .arg(_lastDetectStats.detectPoints)
            .arg(_lastDetectStats.loadedPoints)
            .arg(_lastDetectStats.inlierSum)
            .arg(_lastDetectStats.remainingPoints)
            .arg(accounted));
        summary->setFlags(Qt::ItemIsEnabled);
        summary->setData(0, Qt::UserRole, kRoleSection);
    }

    std::array<int, DetectedShape::TYPE_COUNT> typeCounts{};
    for (const auto& shape : _detectedShapes) {
        const int ti = static_cast<int>(shape.type);
        if (ti >= 0 && ti < static_cast<int>(DetectedShape::TYPE_COUNT)) {
            typeCounts[static_cast<size_t>(ti)]++;
        }
    }

    auto* recognizedRoot = new QTreeWidgetItem(_resultsTree);
    recognizedRoot->setText(0, QString(u8"已识别 (%1)").arg(_detectedShapes.size()));
    recognizedRoot->setFlags(Qt::ItemIsEnabled);
    recognizedRoot->setExpanded(true);
    recognizedRoot->setData(0, Qt::UserRole, kRoleSection);

    for (const auto& cat : kShapeCategories) {
        const int count = typeCounts[static_cast<size_t>(cat.type)];
        if (count <= 0) {
            continue;
        }

        auto* categoryItem = new QTreeWidgetItem(recognizedRoot);
        categoryItem->setText(0, QString(u8"%1 (%2)").arg(QString::fromUtf8(cat.name)).arg(count));
        categoryItem->setData(0, Qt::UserRole, kRoleCategory);
        categoryItem->setFlags(categoryItem->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        categoryItem->setCheckState(0, Qt::Checked);
        categoryItem->setExpanded(true);

        int typeIndex = 1;
        for (size_t i = 0; i < _detectedShapes.size(); ++i) {
            if (_detectedShapes[i].type != cat.type) {
                continue;
            }
            const auto& shape = _detectedShapes[i];
            QString label = QString(u8"%1 #%2  (%3 内点)")
                .arg(QString::fromUtf8(cat.name))
                .arg(typeIndex++)
                .arg(shape.inlierCount);

            auto* item = new QTreeWidgetItem(categoryItem);
            item->setText(0, label);
            item->setData(0, Qt::UserRole, static_cast<int>(i));
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable | Qt::ItemIsEnabled);
            item->setCheckState(0, Qt::Checked);
        }
    }

    if (_remainingPoints && !_remainingPoints->points.empty()) {
        auto* unrecognizedRoot = new QTreeWidgetItem(_resultsTree);
        unrecognizedRoot->setText(0, QString(u8"未识别 (%1 点)")
            .arg(_remainingPoints->points.size()));
        unrecognizedRoot->setFlags(Qt::ItemIsEnabled);
        unrecognizedRoot->setExpanded(true);
        unrecognizedRoot->setData(0, Qt::UserRole, kRoleSection);

        auto* remItem = new QTreeWidgetItem(unrecognizedRoot);
        remItem->setText(0, QString(u8"未分类点云  (%1 点)")
            .arg(_remainingPoints->points.size()));
        remItem->setData(0, Qt::UserRole, kRoleRemaining);
        remItem->setFlags(remItem->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        remItem->setCheckState(0, Qt::Checked);
    }
}

void MainWindow::clearAll()
{
    if (_busy) return;
    _currentCloud.reset(new PointCloudT);
    _detectedShapes.clear();
    _remainingPoints.reset(new PointCloudT);
    _lastDetectStats = {};
    _sourceBaseName.clear();
    _resultRunId.clear();
    _lastResultDir.clear();
    _resultsTree->clear();
    _viewerWidget->clear();
    _infoLabel->clear();
    _infoLabel->setStyleSheet("");
    _detailLabel->clear();
    logMessage(u8"已清除所有数据");
    updateStatus(u8"就绪");
}

void MainWindow::exportResults()
{
    if (_busy) return;
    if (_detectedShapes.empty()) {
        logWarning(u8"没有可导出的结果");
        return;
    }
    _viewerWidget->pauseRendering();
    QString dir = QFileDialog::getExistingDirectory(this, u8"选择导出目录",
        _lastResultDir.isEmpty() ? _lastLoadDir : _lastResultDir);
    _viewerWidget->resumeRendering();
    if (dir.isEmpty()) return;

    QDir outDir(dir);
    for (size_t i = 0; i < _detectedShapes.size(); ++i) {
        QString path = outDir.filePath(shapeFileBaseName(i) + ".pcd");
        PointCloudGenerator::saveToPCD(_detectedShapes[i].inliers, path.toStdWString());
    }
    if (_remainingPoints) {
        QString path = outDir.filePath(_sourceBaseName + "_remaining_unclassified.pcd");
        PointCloudGenerator::saveToPCD(_remainingPoints, path.toStdWString());
    }
    logSuccess(QString(u8"已导出到: %1").arg(outDir.absolutePath()));
}

void MainWindow::setAllRecognizedChecked(Qt::CheckState state)
{
    QSignalBlocker blocker(_resultsTree);
    for (int i = 0; i < _resultsTree->topLevelItemCount(); ++i) {
        QTreeWidgetItem* root = _resultsTree->topLevelItem(i);
        if (root->data(0, Qt::UserRole).toInt() != kRoleSection) {
            continue;
        }
        if (root->text(0).startsWith(u8"已识别")) {
            foreachShapeTreeItem(root, [&](QTreeWidgetItem* item, int idx) {
                item->setCheckState(0, state);
                _viewerWidget->setShapeVisible(idx, state == Qt::Checked);
            });
            break;
        }
    }
    if (state == Qt::Unchecked) {
        _viewerWidget->setHighlightIndex(-1);
    }
}

void MainWindow::onTreeItemSelected(QTreeWidgetItem* item)
{
    if (!item) {
        return;
    }

    const int role = item->data(0, Qt::UserRole).toInt();
    if (role == kRoleSection || role == kRoleCategory) {
        return;
    }

    if (role == kRoleRemaining) {
        _detailLabel->setText(QString(u8"未识别点: %1 个")
            .arg(_remainingPoints ? _remainingPoints->points.size() : 0));
        _viewerWidget->setHighlightIndex(-1);
        return;
    }

    if (role < 0 || role >= static_cast<int>(_detectedShapes.size())) {
        return;
    }

    const auto& shape = _detectedShapes[static_cast<size_t>(role)];
    _detailLabel->setText(QString::fromStdString(shape.getDescription()));

    QSignalBlocker blocker(_resultsTree);
    item->setCheckState(0, Qt::Checked);
    _viewerWidget->setShapeVisible(role, true);
    _viewerWidget->setHighlightIndex(role);
}

void MainWindow::onTreeItemChanged(QTreeWidgetItem* item, int column)
{
    Q_UNUSED(column);
    if (!item) {
        return;
    }

    const int role = item->data(0, Qt::UserRole).toInt();
    if (role == kRoleSection) {
        return;
    }

    const bool visible = (item->checkState(0) == Qt::Checked);

    if (role == kRoleCategory) {
        QSignalBlocker blocker(_resultsTree);
        foreachShapeTreeItem(item, [&](QTreeWidgetItem* child, int idx) {
            child->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
            _viewerWidget->setShapeVisible(idx, visible);
        });
        if (!visible) {
            _viewerWidget->setHighlightIndex(-1);
        }
        return;
    }

    if (role == kRoleRemaining) {
        _viewerWidget->setRemainingVisible(visible);
        return;
    }

    if (role >= 0) {
        _viewerWidget->setShapeVisible(role, visible);
        if (visible) {
            _viewerWidget->setHighlightIndex(role);
        }
    }
}

void MainWindow::showAllShapes()
{
    setAllRecognizedChecked(Qt::Checked);
    _viewerWidget->showAllShapes();
    _detailLabel->clear();
}

void MainWindow::deselectAllShapes()
{
    setAllRecognizedChecked(Qt::Unchecked);
    _detailLabel->clear();
}

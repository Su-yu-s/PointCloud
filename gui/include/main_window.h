#pragma once

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QTreeWidget>
#include <QGroupBox>
#include <QSettings>
#include <QDir>
#include <QFutureWatcher>
#include <QElapsedTimer>
#include <QTimer>
#include <QComboBox>
#include <vector>
#include <cstdint>
#include "point_cloud_viewer_widget.h"
#include "ransac_detector.h"

struct DetectionResult {
    std::vector<DetectedShape> shapes;
    PointCloudT::Ptr remaining;
    RansacDetector::DetectStats stats;
};

struct LoadResult {
    PointCloudT::Ptr cloud;
    bool success = false;
    bool streamDownsampled = false;
    uint64_t sourceBytes = 0;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void generatePointCloud();
    void savePointCloud();
    void loadPointCloud();
    void detectShapes();
    void clearAll();
    void exportResults();
    void onTreeItemSelected(QTreeWidgetItem *item);
    void onTreeItemChanged(QTreeWidgetItem *item, int column);
    void showAllShapes();
    void deselectAllShapes();
    void onDetectionFinished();
    void onLoadFinished();
    void onLoadProgressTick();
    void onGenerateFinished();
    void onSaveFinished();

private:
    void setupUI();
    void setupMenus();
    void logMessage(const QString &message);
    void logSuccess(const QString &message);
    void logWarning(const QString &message);
    void logError(const QString &message);
    void updateStatus(const QString &status);
    void populateResultsTree();
    void setBusy(bool busy, const QString &message = QString());
    QString shapeFileBaseName(size_t index) const;
    QString resultOutputDir() const;
    void setAllRecognizedChecked(Qt::CheckState state);

    PointCloudViewerWidget *_viewerWidget = nullptr;
    QTextEdit *_logText = nullptr;
    QTreeWidget *_resultsTree = nullptr;
    QLabel *_infoLabel = nullptr;
    QLabel *_detailLabel = nullptr;
    QPushButton *_generateBtn = nullptr;
    QPushButton *_loadBtn = nullptr;
    QPushButton *_saveBtn = nullptr;
    QPushButton *_detectBtn = nullptr;
    QComboBox *_detectPresetCombo = nullptr;
    QPushButton *_clearBtn = nullptr;
    QPushButton *_exportBtn = nullptr;
    QFutureWatcher<DetectionResult> *_detectWatcher = nullptr;
    QFutureWatcher<LoadResult> *_loadWatcher = nullptr;
    QFutureWatcher<PointCloudT::Ptr> *_generateWatcher = nullptr;
    QFutureWatcher<void> *_saveWatcher = nullptr;
    PointCloudT::Ptr _currentCloud;
    std::vector<DetectedShape> _detectedShapes;
    PointCloudT::Ptr _remainingPoints;
    RansacDetector::DetectStats _lastDetectStats;
    QString _sourceBaseName;
    QString _resultRunId;
    QString _lastSaveDir;
    QString _lastLoadDir;
    QString _lastResultDir;
    bool _busy = false;
    QElapsedTimer _detectTimer;
    QTimer* _loadProgressTimer = nullptr;
};

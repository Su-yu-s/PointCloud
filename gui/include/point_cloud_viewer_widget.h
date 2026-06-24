#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QTimer>
#include <QFrame>
#include <QLabel>
#include <QProgressBar>
#include <QPoint>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector3D>
#include <vector>
#include "ransac_detector.h"

class PointCloudViewerWidget : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT

public:
    explicit PointCloudViewerWidget(QWidget *parent = nullptr);
    ~PointCloudViewerWidget() override;

    void setPointCloud(PointCloudT::Ptr cloud);
    void showDetectionResults(const std::vector<DetectedShape>& shapes,
                              PointCloudT::Ptr remaining,
                              PointCloudT::Ptr baseCloud);
    void setShapeVisible(int index, bool visible);
    void setRemainingVisible(bool visible);
    void setHighlightIndex(int index);
    void showAllShapes();
    void showOnlyShape(int index);
    void clear();
    void pauseRendering();
    void resumeRendering();
    void setTaskActive(bool active, const QString& message = QString());
    void setTaskMessage(const QString& message);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    struct GpuCloud {
        QOpenGLBuffer vbo;
        int pointCount = 0;
        bool enabled = true;
    };
    void uploadCloud(GpuCloud& gc, const PointCloudT::Ptr& cloud, uint8_t r, uint8_t g, uint8_t b);
    void releaseCloud(GpuCloud& gc);
    void drawCloud(GpuCloud& gc, float brightness = 1.f, float alpha = 1.f);
    static void shapeColorForIndex(size_t index, uint8_t& r, uint8_t& g, uint8_t& b);
    void updateSoloFromVisibility();
    void updateCamera();
    void updatePointSizesFromSpan();
    QVector3D arcballVector(const QPoint& pos) const;

    QOpenGLShaderProgram _prog;
    QOpenGLVertexArrayObject _vao;
    int _attrPos = -1, _attrCol = -1;
    int _uniMVP = -1;
    int _uniPointSize = -1;
    int _uniBrightness = -1;
    int _uniAlpha = -1;

    // 相机
    QMatrix4x4 _view, _proj;
    QQuaternion _rotation;
    QVector3D _center{0, 0, 0};
    float _distance = 10.0f;
    float _spanHint = 1.0f;
    QPoint _lastMouse;
    bool _rotating = false, _panning = false;

    // GPU 云
    GpuCloud _baseGpu, _remainingGpu, _bgGpu;
    std::vector<GpuCloud> _shapeGpus;
    std::vector<bool> _shapeVisible;
    bool _remainingVisible = false;
    bool _soloMode = false;
    int _highlightIndex = -1;
    int _currentPointSize = 4;
    int _bgPointSize = 4;
    int _shapePointSize = 6;

    // 状态
    PointCloudT::Ptr _baseCloud;
    std::vector<DetectedShape> _shapes;
    PointCloudT::Ptr _remaining;
    QTimer _timer;
    bool _dirty = false;
    bool _glReady = false;

    // 状态栏
    QFrame* _statusBar = nullptr;
    QLabel* _statusLabel = nullptr;
    QProgressBar* _statusProgress = nullptr;
    void layoutStatusBar();

    static const uint8_t COLORS[10][3];
};

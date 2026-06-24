#include "point_cloud_viewer_widget.h"
#include "point_cloud_generator.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QHBoxLayout>
#include <algorithm>
#include <cmath>
#include <iostream>

static const char* kVertShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aCol;
uniform mat4 uMVP;
uniform float uPointSize;
out vec3 vCol;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    gl_PointSize = uPointSize;
    vCol = aCol;
}
)";

static const char* kFragShader = R"(
#version 330 core
in vec3 vCol;
uniform float uBrightness;
uniform float uAlpha;
out vec4 fragColor;
void main() {
    vec2 d = gl_PointCoord - vec2(0.5);
    float r2 = dot(d, d);
    if (r2 > 0.25) {
        discard;
    }
    float alpha = smoothstep(0.25, 0.08, r2);
    vec3 col = pow(clamp(vCol * uBrightness, 0.0, 1.0), vec3(0.88));
    fragColor = vec4(col, alpha * uAlpha);
}
)";

namespace {

float safeSpan(const PointCloudT::Ptr& cloud) {
    return PointCloudGenerator::estimateSpanSampled(cloud, 5000);
}

QVector3D cloudCenter(const PointCloudT::Ptr& cloud) {
    if (!cloud || cloud->points.empty()) return {0,0,0};
    double sx=0, sy=0, sz=0;
    for (const auto& p : cloud->points) { sx+=p.x; sy+=p.y; sz+=p.z; }
    auto n = static_cast<double>(cloud->points.size());
    return {static_cast<float>(sx/n), static_cast<float>(sy/n), static_cast<float>(sz/n)};
}

} // namespace

const uint8_t PointCloudViewerWidget::COLORS[][3] = {
    {239,68,68},{16,185,129},{59,130,246},{245,158,11},{168,85,247},
    {0,217,255},{236,72,153},{34,197,94},{99,102,241},{251,146,60}
};

PointCloudViewerWidget::PointCloudViewerWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    setMinimumSize(400, 300);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);

    connect(&_timer, &QTimer::timeout, this, [this]() {
        if (_dirty && _glReady) {
            update();
        }
    });
    _timer.start(30);

    _statusBar = new QFrame(this);
    _statusBar->setObjectName("viewerStatusBar");
    _statusBar->hide();
    _statusBar->setStyleSheet(R"(
        QFrame#viewerStatusBar{background:#fff;border-bottom:1px solid #e2e8f0;}
        QLabel{color:#64748b;font-size:12px;padding-left:2px;}
        QProgressBar{background:#e2e8f0;border:none;border-radius:1px;min-height:2px;max-height:2px;}
        QProgressBar::chunk{background:#2563eb;border-radius:1px;}
    )");
    auto* bl = new QHBoxLayout(_statusBar);
    bl->setContentsMargins(12,0,12,0); bl->setSpacing(12);
    _statusLabel = new QLabel(_statusBar);
    _statusLabel->setObjectName("viewerStatusLabel");
    _statusProgress = new QProgressBar(_statusBar);
    _statusProgress->setObjectName("viewerStatusProgress");
    _statusProgress->setTextVisible(false);
    _statusProgress->setRange(0,0);
    _statusProgress->setFixedHeight(2);
    bl->addWidget(_statusLabel, 0, Qt::AlignVCenter);
    bl->addWidget(_statusProgress, 1, Qt::AlignVCenter);
}

PointCloudViewerWidget::~PointCloudViewerWidget() {
    makeCurrent();
    releaseCloud(_baseGpu); releaseCloud(_remainingGpu); releaseCloud(_bgGpu);
    for (auto& g : _shapeGpus) releaseCloud(g);
    _vao.destroy(); _prog.removeAllShaders();
    doneCurrent();
}

void PointCloudViewerWidget::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.22f, 0.24f, 0.28f, 1.0f);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    _prog.addShaderFromSourceCode(QOpenGLShader::Vertex, kVertShader);
    _prog.addShaderFromSourceCode(QOpenGLShader::Fragment, kFragShader);
    if (!_prog.link()) {
        std::cerr << u8"[Viewer] Shader link failed: "
                  << _prog.log().toStdString() << std::endl;
    }
    _prog.bind();
    _attrPos = _prog.attributeLocation("aPos");
    _attrCol = _prog.attributeLocation("aCol");
    _uniMVP = _prog.uniformLocation("uMVP");
    _uniPointSize = _prog.uniformLocation("uPointSize");
    _uniBrightness = _prog.uniformLocation("uBrightness");
    _uniAlpha = _prog.uniformLocation("uAlpha");
    _prog.release();

    _vao.create();
    _glReady = true;
}

void PointCloudViewerWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    layoutStatusBar();
    updateCamera();
}

void PointCloudViewerWidget::paintGL() {
    _dirty = false;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!_glReady || _prog.isLinked() == false) {
        return;
    }

    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const QMatrix4x4 mvp = _proj * _view;
    _prog.bind();
    _prog.setUniformValue(_uniMVP, mvp);
    _vao.bind();

    if (_bgGpu.enabled && _bgGpu.pointCount > 0) {
        // 底云始终用检测前同款白色；未勾选的几何体不再叠彩色，自然恢复为白点
        _prog.setUniformValue(_uniPointSize, static_cast<float>(_currentPointSize));
        const float bgBright = _soloMode ? 0.38f : 1.f;
        const float bgAlpha = _soloMode ? 0.5f : 1.f;
        drawCloud(_bgGpu, bgBright, bgAlpha);
    }
    if (_baseGpu.enabled && _baseGpu.pointCount > 0) {
        _prog.setUniformValue(_uniPointSize, static_cast<float>(_currentPointSize));
        drawCloud(_baseGpu);
    }
    for (size_t i = 0; i < _shapeGpus.size(); ++i) {
        if ((i < _shapeVisible.size() && !_shapeVisible[i]) || !_shapeGpus[i].enabled) {
            continue;
        }
        const bool highlighted = (static_cast<int>(i) == _highlightIndex);
        int ps = _shapePointSize;
        float bright = 1.f;
        float alpha = 0.94f;
        if (_soloMode && highlighted) {
            ps += 7;
            bright = 1.28f;
            alpha = 1.f;
        } else if (highlighted) {
            ps += 4;
            bright = 1.16f;
            alpha = 1.f;
        }
        _prog.setUniformValue(_uniPointSize, static_cast<float>(ps));
        drawCloud(_shapeGpus[i], bright, alpha);
    }
    if (_remainingVisible && _remainingGpu.enabled && _remainingGpu.pointCount > 0) {
        _prog.setUniformValue(_uniPointSize, static_cast<float>(_currentPointSize));
        drawCloud(_remainingGpu, 0.88f, 0.85f);
    }

    _vao.release();
    _prog.release();
}

void PointCloudViewerWidget::drawCloud(GpuCloud& gc, float brightness, float alpha) {
    _prog.setUniformValue(_uniBrightness, brightness);
    _prog.setUniformValue(_uniAlpha, alpha);
    gc.vbo.bind();
    _prog.setAttributeBuffer(_attrPos, GL_FLOAT, 0, 3, 6*sizeof(float));
    _prog.enableAttributeArray(_attrPos);
    _prog.setAttributeBuffer(_attrCol, GL_FLOAT, 3*sizeof(float), 3, 6*sizeof(float));
    _prog.enableAttributeArray(_attrCol);
    glDrawArrays(GL_POINTS, 0, gc.pointCount);
    gc.vbo.release();
}

void PointCloudViewerWidget::shapeColorForIndex(size_t index, uint8_t& r, uint8_t& g, uint8_t& b) {
    constexpr float golden = 0.618033988749895f;
    const float h = std::fmod(static_cast<float>(index) * golden, 1.f);
    const float s = 0.72f;
    const float v = 0.95f;
    const float c = v * s;
    const float hp = h * 6.f;
    const float x = c * (1.f - std::fabs(std::fmod(hp, 2.f) - 1.f));
    const float m = v - c;
    float r1 = 0.f, g1 = 0.f, b1 = 0.f;
    switch (static_cast<int>(hp) % 6) {
    case 0: r1 = c; g1 = x; b1 = 0.f; break;
    case 1: r1 = x; g1 = c; b1 = 0.f; break;
    case 2: r1 = 0.f; g1 = c; b1 = x; break;
    case 3: r1 = 0.f; g1 = x; b1 = c; break;
    case 4: r1 = x; g1 = 0.f; b1 = c; break;
    default: r1 = c; g1 = 0.f; b1 = x; break;
    }
    r = static_cast<uint8_t>(std::min(255.f, (r1 + m) * 255.f));
    g = static_cast<uint8_t>(std::min(255.f, (g1 + m) * 255.f));
    b = static_cast<uint8_t>(std::min(255.f, (b1 + m) * 255.f));
}

void PointCloudViewerWidget::updateSoloFromVisibility() {
    int only = -1;
    int count = 0;
    for (size_t i = 0; i < _shapeVisible.size(); ++i) {
        if (!_shapeVisible[i]) {
            continue;
        }
        only = static_cast<int>(i);
        ++count;
    }
    _soloMode = (count == 1);
    if (_soloMode) {
        _highlightIndex = only;
    }
}

void PointCloudViewerWidget::uploadCloud(GpuCloud& gc, const PointCloudT::Ptr& cloud, uint8_t r, uint8_t g, uint8_t b) {
    releaseCloud(gc);
    if (!cloud || cloud->points.empty()) return;

    const size_t n = cloud->points.size();
    const size_t step = (n > 850000) ? std::max<size_t>(1, n / 750000) : 1;
    gc.pointCount = static_cast<int>((n + step - 1) / step);

    const float cr = r / 255.f;
    const float cg = g / 255.f;
    const float cb = b / 255.f;
    std::vector<float> data;
    data.reserve(static_cast<size_t>(gc.pointCount) * 6);
    for (size_t i = 0; i < n; i += step) {
        const auto& p = cloud->points[i];
        data.push_back(p.x);
        data.push_back(p.y);
        data.push_back(p.z);
        data.push_back(cr);
        data.push_back(cg);
        data.push_back(cb);
    }
    gc.vbo.create();
    gc.vbo.bind();
    gc.vbo.allocate(data.data(), static_cast<int>(data.size() * sizeof(float)));
    gc.vbo.release();
    gc.enabled = true;
}

void PointCloudViewerWidget::releaseCloud(GpuCloud& gc) {
    if (gc.vbo.isCreated()) gc.vbo.destroy();
    gc.pointCount = 0; gc.enabled = false;
}

// -- 相机 --
void PointCloudViewerWidget::updatePointSizesFromSpan() {
    int base = 4;
    if (_spanHint > 0 && width() > 0) {
        const float pxPerUnit = static_cast<float>(width()) / _spanHint;
        if (pxPerUnit < 1.5f) {
            base = 6;
        } else if (pxPerUnit < 3.f) {
            base = 5;
        } else if (pxPerUnit < 6.f) {
            base = 4;
        } else {
            base = 3;
        }
    }
    _currentPointSize = base;
    _bgPointSize = base + 1;
    _shapePointSize = base + 2;
}

void PointCloudViewerWidget::updateCamera() {
    float aspect = static_cast<float>(width()) / std::max(height(), 1);
    _proj.setToIdentity();
    _proj.perspective(45.f, aspect, _spanHint * 0.001f, _spanHint * 20.f);

    QMatrix4x4 rot;
    rot.rotate(_rotation);
    QVector3D eye = rot * QVector3D(0, 0, _distance);
    QVector3D up = rot * QVector3D(0, 1, 0);
    _view.setToIdentity();
    _view.lookAt(eye + _center, _center, up);
    updatePointSizesFromSpan();
}

QVector3D PointCloudViewerWidget::arcballVector(const QPoint& pos) const {
    float x = (2.f * pos.x() / std::max(width(), 1)) - 1.f;
    float y = 1.f - (2.f * pos.y() / std::max(height(), 1));
    float zz = 1.f - x*x - y*y;
    float z = zz > 0 ? std::sqrt(zz) : 0;
    QVector3D v(x, y, z); v.normalize(); return v;
}

void PointCloudViewerWidget::mousePressEvent(QMouseEvent *e) {
    if (e->button() == Qt::LeftButton) {
        _rotating = true;
        _lastMouse = e->pos();
    }
    if (e->button() == Qt::MiddleButton || e->button() == Qt::RightButton) {
        _panning = true;
        _lastMouse = e->pos();
    }
}

void PointCloudViewerWidget::mouseReleaseEvent(QMouseEvent *e) {
    if (e->button() == Qt::LeftButton) {
        _rotating = false;
    }
    if (e->button() == Qt::MiddleButton || e->button() == Qt::RightButton) {
        _panning = false;
    }
}

void PointCloudViewerWidget::mouseMoveEvent(QMouseEvent *e) {
    if (_rotating) {
        QVector3D v0 = arcballVector(_lastMouse);
        QVector3D v1 = arcballVector(e->pos());
        float dot = QVector3D::dotProduct(v0, v1);
        if (dot < 0.9999f) {
            QVector3D axis = QVector3D::crossProduct(v0, v1).normalized();
            float angle = std::acos(std::max(-1.f, std::min(1.f, dot)));
            _rotation = QQuaternion::fromAxisAndAngle(axis, angle * 180.f / 3.14159265f) * _rotation;
        }
        updateCamera();
        _dirty = true;
        update();
    } else if (_panning) {
        float s = _spanHint / std::max(width(), 1) * 2.f;
        QMatrix4x4 rot; rot.rotate(_rotation);
        QVector3D right = rot * QVector3D(s,0,0);
        QVector3D up    = rot * QVector3D(0,-s,0);
        _center += right * (e->pos().x() - _lastMouse.x());
        _center += up    * (e->pos().y() - _lastMouse.y());
        updateCamera();
        _dirty = true;
        update();
    }
    _lastMouse = e->pos();
}

void PointCloudViewerWidget::wheelEvent(QWheelEvent *e) {
    _distance *= (e->angleDelta().y() > 0) ? 0.9f : 1.1f;
    _distance = std::max(_spanHint * 0.1f, std::min(_distance, _spanHint * 30.f));
    updateCamera();
    _dirty = true;
    update();
}

// -- 公共接口 --
void PointCloudViewerWidget::setPointCloud(PointCloudT::Ptr cloud) {
    makeCurrent();
    _baseCloud = cloud; _shapes.clear(); _remaining.reset();
    _shapeVisible.clear(); _shapeGpus.clear(); _highlightIndex = -1; _soloMode = false;
    releaseCloud(_baseGpu); releaseCloud(_bgGpu); releaseCloud(_remainingGpu);

    if (cloud && !cloud->points.empty()) {
        _spanHint = safeSpan(cloud);
        _center = cloudCenter(cloud);
        _distance = _spanHint * 1.5f;
        updatePointSizesFromSpan();
        uploadCloud(_baseGpu, cloud, 220, 224, 232);
        _baseGpu.enabled = true;
    }
    _bgGpu.enabled = false;
    _remainingGpu.enabled = false;
    updateCamera(); _dirty = true;
    doneCurrent();
}

void PointCloudViewerWidget::showDetectionResults(const std::vector<DetectedShape>& shapes,
                                           PointCloudT::Ptr remaining,
                                           PointCloudT::Ptr baseCloud) {
    makeCurrent();
    _shapes = shapes; _remaining = remaining; _baseCloud = baseCloud;
    _shapeVisible.assign(shapes.size(), true);
    _remainingVisible = true; _highlightIndex = -1;
    _soloMode = false;
    releaseCloud(_baseGpu); releaseCloud(_bgGpu); releaseCloud(_remainingGpu);
    for (auto& g : _shapeGpus) releaseCloud(g);
    _shapeGpus.clear();
    _shapeGpus.resize(shapes.size());

    if (baseCloud && !baseCloud->points.empty()) {
        _spanHint = safeSpan(baseCloud);
        _center = cloudCenter(baseCloud);
        _distance = _spanHint * 1.5f;
        updatePointSizesFromSpan();
    }

    // 底云（与检测前相同的浅白整云）+ 彩色形状内点 + 剩余点
    _baseGpu.enabled = false;
    if (baseCloud && !baseCloud->points.empty()) {
        uploadCloud(_bgGpu, baseCloud, 220, 224, 232);
        _bgGpu.enabled = true;
    }

    for (size_t i = 0; i < shapes.size(); ++i) {
        if (!shapes[i].inliers || shapes[i].inliers->points.empty()) {
            continue;
        }
        uint8_t cr = 0, cg = 0, cb = 0;
        shapeColorForIndex(i, cr, cg, cb);
        uploadCloud(_shapeGpus[i], shapes[i].inliers, cr, cg, cb);
    }
    if (remaining && !remaining->points.empty()) {
        uploadCloud(_remainingGpu, remaining, 200, 205, 215);
    }
    updateCamera(); _dirty = true;
    doneCurrent();
}

void PointCloudViewerWidget::setShapeVisible(int index, bool visible) {
    if (index >= 0 && index < static_cast<int>(_shapeVisible.size())) {
        _shapeVisible[index] = visible;
        updateSoloFromVisibility();
    }
    _dirty = true;
}

void PointCloudViewerWidget::setRemainingVisible(bool visible) { _remainingVisible = visible; _dirty = true; }

void PointCloudViewerWidget::setHighlightIndex(int index) { _highlightIndex = index; _dirty = true; }

void PointCloudViewerWidget::showAllShapes() {
    _shapeVisible.assign(_shapes.size(), true);
    _remainingVisible = true; _highlightIndex = -1; _soloMode = false; _dirty = true;
}

void PointCloudViewerWidget::showOnlyShape(int index) {
    if (index < 0 || index >= static_cast<int>(_shapes.size())) return;
    _shapeVisible.assign(_shapes.size(), false);
    _shapeVisible[index] = true;
    _remainingVisible = false;
    _highlightIndex = index;
    _soloMode = true;
    _dirty = true;
}

void PointCloudViewerWidget::clear() {
    makeCurrent();
    _baseCloud.reset(); _shapes.clear(); _remaining.reset();
    _shapeVisible.clear(); _highlightIndex = -1; _soloMode = false;
    releaseCloud(_baseGpu); releaseCloud(_bgGpu); releaseCloud(_remainingGpu);
    for (auto& g : _shapeGpus) releaseCloud(g);
    _shapeGpus.clear();
    _dirty = true;
    doneCurrent();
}

void PointCloudViewerWidget::pauseRendering() { _timer.stop(); }
void PointCloudViewerWidget::resumeRendering() { _dirty = true; _timer.start(30); }

void PointCloudViewerWidget::setTaskActive(bool active, const QString& msg) {
    if (!_statusBar) return;
    if (active) {
        if (!msg.isEmpty()) _statusLabel->setText(msg);
        layoutStatusBar(); _statusBar->show(); _statusBar->raise();
        _statusProgress->setRange(0,0);
    } else { _statusBar->hide(); }
}

void PointCloudViewerWidget::setTaskMessage(const QString& msg) {
    if (_statusLabel && _statusBar && _statusBar->isVisible()) _statusLabel->setText(msg);
}

void PointCloudViewerWidget::layoutStatusBar() {
    if (_statusBar) _statusBar->setGeometry(0, 0, width(), 34);
}

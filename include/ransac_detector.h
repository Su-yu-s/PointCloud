#pragma once

#include "point_cloud_generator.h"
#include "schnabel_detector.h"
#include <vector>

class RansacDetector {
public:
    struct DetectStats {
        size_t loadedPoints = 0;
        size_t detectPoints = 0;
        size_t inlierSum = 0;
        size_t remainingPoints = 0;
    };

    struct Config {
        enum class Preset {
            Architecture,  // 游乐场/大场景：仅平面，更高内点门槛
            Full           // 机械/零件：平面+柱+球+锥+环
        };

        Preset preset = Preset::Architecture;

        double planeDistThreshold   = 0.025;
        double sphereDistThreshold  = 0.03;
        double cylinderDistThreshold = 0.05;
        int    maxIterations        = 8000;
        double normalSearchRadius   = 0.10;

        int    minInlierCount       = 100;
        int    maxShapesToDetect    = 15;
        int    maxCylinders         = 3;
        int    maxSpheres           = 3;
        int    maxPlanes            = 10;
        double sphereRadiusMin     = 0.05;
        double sphereRadiusMax     = 2.0;
        double cylinderRadiusMin   = 0.05;
        double cylinderRadiusMax   = 1.5;

        double coneDistThreshold   = 0.07;
        int    maxCones            = 3;
        double coneAngleMin        = 0.1;
        double coneAngleMax        = 1.2;

        double torusDistThreshold  = 0.06;
        int    maxTori             = 2;

        double lineDistThreshold   = 0.03;
        int    maxLines            = 3;

        double clusterTolerance    = 0.12;
        int    minClusterSize      = 80;
        int    maxClusterSize      = 10000;

        double minInlierRatio      = 0.20;
    };

    explicit RansacDetector(const Config& config = Config());

    std::vector<DetectedShape> detect(const PointCloudT::Ptr& inputCloud);

    /** 新接口：Resurfacer 点数组；结果内点存 inlierIndices，不拷贝输入点云 */
    std::vector<DetectedShape> detectFromPoints(const float (*points)[3], int point_num);

    PointCloudT::Ptr getRemainingPoints() const { return remaining_; }
    const std::vector<unsigned>& lastRemainingIndices() const { return remaining_indices_; }
    const DetectStats& lastDetectStats() const { return stats_; }

    void setPlaneDistanceThreshold(double val) { config_.planeDistThreshold = val; }
    void setSphereDistanceThreshold(double val) { config_.sphereDistThreshold = val; }
    void setCylinderDistanceThreshold(double val) { config_.cylinderDistThreshold = val; }
    void setConeDistanceThreshold(double val) { config_.coneDistThreshold = val; }
    void setTorusDistanceThreshold(double val) { config_.torusDistThreshold = val; }
    void setLineDistanceThreshold(double val) { config_.lineDistThreshold = val; }
    void setMaxIterations(int val) { config_.maxIterations = val; }
    void setMinInliers(int val) { config_.minInlierCount = val; }

private:
    Config config_;
    PointCloudT::Ptr remaining_;
    std::vector<unsigned> remaining_indices_;
    DetectStats stats_;

    void autoTuneParameters(const PointCloudT::Ptr& cloud);
    void autoTuneParametersFromPoints(const float (*points)[3], int point_num);
    SchnabelDetector::Params buildSchnabelParams(const PointCloudT::Ptr& cloud) const;
    SchnabelDetector::Params buildSchnabelParamsForCount(size_t pointCount, float span) const;
    std::vector<DetectedShape> filterShapes(std::vector<DetectedShape>&& shapes,
                                            PointCloudT::Ptr& remaining,
                                            float span) const;
    std::vector<DetectedShape> filterShapesIndices(std::vector<DetectedShape>&& shapes,
                                                   std::vector<unsigned>& remaining_indices,
                                                   float span,
                                                   const float (*points)[3]) const;
    void applyPresetMinInliers(size_t pointCount);
};

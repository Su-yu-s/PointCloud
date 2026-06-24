#include "ransac_lib_api.h"

#include "ransac_detector.h"

#include <cstring>
#include <new>
#include <vector>

namespace {

RansacShapeType toApiType(DetectedShape::Type t) {
    switch (t) {
    case DetectedShape::PLANE: return RANSAC_SHAPE_PLANE;
    case DetectedShape::SPHERE: return RANSAC_SHAPE_SPHERE;
    case DetectedShape::CYLINDER: return RANSAC_SHAPE_CYLINDER;
    case DetectedShape::CONE: return RANSAC_SHAPE_CONE;
    case DetectedShape::TORUS: return RANSAC_SHAPE_TORUS;
    default: return RANSAC_SHAPE_UNKNOWN;
    }
}

RansacDetector::Config makeConfig(const RansacDetectParams* params) {
    RansacDetector::Config cfg;
    if (!params) {
        cfg.preset = RansacDetector::Config::Preset::Architecture;
        return cfg;
    }

    switch (params->preset) {
    case RANSAC_PRESET_MECHANICAL_FULL:
        cfg.preset = RansacDetector::Config::Preset::Full;
        break;
    case RANSAC_PRESET_OUTDOOR_PLANES:
    default:
        cfg.preset = RansacDetector::Config::Preset::Architecture;
        break;
    }

    if (params->min_inlier_count > 0) {
        cfg.minInlierCount = params->min_inlier_count;
    }
    return cfg;
}

} // namespace

extern "C" int ransac_detect(const RansacInputCloud* input,
                             const RansacDetectParams* params,
                             RansacDetectOutput* output) {
    if (!input || !input->points || input->point_num <= 0 || !output) {
        return -1;
    }

    std::memset(output, 0, sizeof(*output));

    RansacDetector detector(makeConfig(params));
    const std::vector<DetectedShape> shapes =
        detector.detectFromPoints(input->points, input->point_num);
    const std::vector<unsigned>& remaining = detector.lastRemainingIndices();
    const RansacDetector::DetectStats stats = detector.lastDetectStats();

    output->input_point_count = static_cast<int>(stats.loadedPoints);
    output->inlier_point_sum = static_cast<int>(stats.inlierSum);
    output->shape_count = static_cast<int>(shapes.size());
    output->remaining_count = static_cast<int>(remaining.size());

    if (output->shape_count > 0) {
        output->shapes = new RansacShapeItem[static_cast<size_t>(output->shape_count)];
        for (int i = 0; i < output->shape_count; ++i) {
            const DetectedShape& src = shapes[static_cast<size_t>(i)];
            RansacShapeItem& dst = output->shapes[i];
            dst.type = toApiType(src.type);
            dst.inlier_count = src.inlierCount;

            if (dst.inlier_count > 0 && !src.inlierIndices.empty()) {
                dst.inlier_indices = new int[static_cast<size_t>(dst.inlier_count)];
                for (int j = 0; j < dst.inlier_count; ++j) {
                    dst.inlier_indices[j] = static_cast<int>(src.inlierIndices[static_cast<size_t>(j)]);
                }
            } else {
                dst.inlier_indices = nullptr;
            }

            if (src.coefficients && !src.coefficients->values.empty()) {
                dst.coefficient_count = static_cast<int>(src.coefficients->values.size());
                dst.coefficients = new float[static_cast<size_t>(dst.coefficient_count)];
                for (int k = 0; k < dst.coefficient_count; ++k) {
                    dst.coefficients[k] = src.coefficients->values[static_cast<size_t>(k)];
                }
            } else {
                dst.coefficient_count = 0;
                dst.coefficients = nullptr;
            }
        }
    }

    if (output->remaining_count > 0) {
        output->remaining_indices = new int[static_cast<size_t>(output->remaining_count)];
        for (int i = 0; i < output->remaining_count; ++i) {
            output->remaining_indices[i] = static_cast<int>(remaining[static_cast<size_t>(i)]);
        }
    }

    return 0;
}

extern "C" void ransac_free_output(RansacDetectOutput* output) {
    if (!output) {
        return;
    }

    if (output->shapes) {
        for (int i = 0; i < output->shape_count; ++i) {
            delete[] output->shapes[i].inlier_indices;
            delete[] output->shapes[i].coefficients;
        }
        delete[] output->shapes;
    }
    delete[] output->remaining_indices;

    std::memset(output, 0, sizeof(*output));
}

extern "C" const char* ransac_shape_type_name(RansacShapeType type) {
    switch (type) {
    case RANSAC_SHAPE_PLANE: return "Plane";
    case RANSAC_SHAPE_SPHERE: return "Sphere";
    case RANSAC_SHAPE_CYLINDER: return "Cylinder";
    case RANSAC_SHAPE_CONE: return "Cone";
    case RANSAC_SHAPE_TORUS: return "Torus";
    default: return "Unknown";
    }
}

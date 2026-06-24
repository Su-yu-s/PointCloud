#include "point_cloud_io.h"

#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string trim(const std::string& s) {
    size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) {
        ++b;
    }
    size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {
        --e;
    }
    return s.substr(b, e - b);
}

bool parseHeaderLine(const std::string& line, std::string& key, std::string& value) {
    const auto pos = line.find(' ');
    if (pos == std::string::npos) {
        return false;
    }
    key = trim(line.substr(0, pos));
    value = trim(line.substr(pos + 1));
    return true;
}

} // namespace

namespace PointCloudIO {

bool savePCDBinary(const PointCloudT::Ptr& cloud, const std::string& path) {
    if (!cloud) {
        return false;
    }
    FILE* fp = fopen(path.c_str(), "wb");
    if (!fp) {
        return false;
    }

    const size_t n = cloud->points.size();
    fprintf(fp, "# .PCD v0.7 - Point Cloud Data file format\n");
    fprintf(fp, "VERSION 0.7\n");
    fprintf(fp, "FIELDS x y z\n");
    fprintf(fp, "SIZE 4 4 4\n");
    fprintf(fp, "TYPE F F F\n");
    fprintf(fp, "COUNT 1 1 1\n");
    fprintf(fp, "WIDTH %zu\n", n);
    fprintf(fp, "HEIGHT 1\n");
    fprintf(fp, "VIEWPOINT 0 0 0 1 0 0 0\n");
    fprintf(fp, "POINTS %zu\n", n);
    fprintf(fp, "DATA binary\n");

    for (const auto& p : cloud->points) {
        fwrite(&p.x, sizeof(float), 1, fp);
        fwrite(&p.y, sizeof(float), 1, fp);
        fwrite(&p.z, sizeof(float), 1, fp);
    }

    fclose(fp);
    return true;
}

bool loadPCD(const std::string& path, PointCloudT::Ptr& cloud) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }

    std::string line;
    size_t points = 0;
    bool binary = false;
    std::streampos dataPos = 0;

    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::string key;
        std::string value;
        if (!parseHeaderLine(line, key, value)) {
            continue;
        }
        if (key == "POINTS") {
            points = static_cast<size_t>(std::stoul(value));
        } else if (key == "DATA") {
            binary = (value == "binary");
            dataPos = in.tellg();
            break;
        }
    }

    if (points == 0) {
        return false;
    }

    cloud = PointCloudT::Ptr(new SimplePointCloud);
    cloud->points.reserve(points);

    if (binary) {
        in.seekg(dataPos);
        for (size_t i = 0; i < points; ++i) {
            Point3f p;
            in.read(reinterpret_cast<char*>(&p.x), sizeof(float));
            in.read(reinterpret_cast<char*>(&p.y), sizeof(float));
            in.read(reinterpret_cast<char*>(&p.z), sizeof(float));
            if (!in) {
                break;
            }
            cloud->points.push_back(p);
        }
    } else {
        while (std::getline(in, line) && cloud->points.size() < points) {
            if (line.empty()) {
                continue;
            }
            Point3f p;
            if (sscanf(line.c_str(), "%f %f %f", &p.x, &p.y, &p.z) == 3) {
                cloud->points.push_back(p);
            }
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
    return !cloud->points.empty();
}

bool loadPLY(const std::string& path, PointCloudT::Ptr& cloud) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }

    std::string line;
    if (!std::getline(in, line) || trim(line) != "ply") {
        return false;
    }

    bool ascii = true;
    size_t vertexCount = 0;
    std::vector<std::string> props;

    in.clear();
    in.seekg(0);
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.rfind("format ", 0) == 0) {
            ascii = (line.find("ascii") != std::string::npos);
        } else if (line.rfind("element vertex ", 0) == 0) {
            vertexCount = static_cast<size_t>(std::stoul(line.substr(15)));
        } else if (line.rfind("property ", 0) == 0) {
            props.push_back(trim(line.substr(9)));
        } else if (line == "end_header") {
            break;
        }
    }
    if (vertexCount == 0) {
        return false;
    }

    int ix = -1, iy = -1, iz = -1;
    for (size_t i = 0; i < props.size(); ++i) {
        const auto& prop = props[i];
        if (prop == "float x" || prop == "double x") {
            ix = static_cast<int>(i);
        } else if (prop == "float y" || prop == "double y") {
            iy = static_cast<int>(i);
        } else if (prop == "float z" || prop == "double z") {
            iz = static_cast<int>(i);
        }
    }
    if (ix < 0 || iy < 0 || iz < 0) {
        return false;
    }

    cloud = PointCloudT::Ptr(new SimplePointCloud);
    cloud->points.reserve(vertexCount);

    if (ascii) {
        for (size_t i = 0; i < vertexCount; ++i) {
            if (!std::getline(in, line)) {
                break;
            }
            std::istringstream iss(line);
            std::vector<double> vals;
            double v = 0.0;
            while (iss >> v) {
                vals.push_back(v);
            }
            if (static_cast<int>(vals.size()) <= std::max({ix, iy, iz})) {
                continue;
            }
            Point3f p;
            p.x = static_cast<float>(vals[static_cast<size_t>(ix)]);
            p.y = static_cast<float>(vals[static_cast<size_t>(iy)]);
            p.z = static_cast<float>(vals[static_cast<size_t>(iz)]);
            cloud->points.push_back(p);
        }
    } else {
        for (size_t i = 0; i < vertexCount; ++i) {
            std::vector<float> vals(props.size(), 0.f);
            for (size_t j = 0; j < props.size(); ++j) {
                in.read(reinterpret_cast<char*>(&vals[j]), sizeof(float));
            }
            if (!in) {
                break;
            }
            Point3f p;
            p.x = vals[static_cast<size_t>(ix)];
            p.y = vals[static_cast<size_t>(iy)];
            p.z = vals[static_cast<size_t>(iz)];
            cloud->points.push_back(p);
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
    return !cloud->points.empty();
}

} // namespace PointCloudIO

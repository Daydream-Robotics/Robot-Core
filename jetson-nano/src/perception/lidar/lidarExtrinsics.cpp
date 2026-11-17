#include "perception/lidar/lidarExtrinsics.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

constexpr double kPi = 3.14159265358979323846264338327950288;
constexpr double kRadiansPerDegree = kPi / 180.0;
constexpr double kAngleEpsilon = 1e-12;

std::string trim(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string removeComment(const std::string& line) {
    const auto hash = line.find('#');
    const auto slash = line.find("//");
    const auto end = std::min(hash, slash); // npos sorts after valid positions.
    return trim(line.substr(0, end));
}

double parseDouble(const std::string& text) {
    const std::string input = trim(text);
    if (input.empty()) throw std::runtime_error("missing numeric value");
    std::size_t used = 0;
    const double result = std::stod(input, &used);
    if (used != input.size() || !std::isfinite(result)) {
        throw std::runtime_error("invalid finite number: '" + input + "'");
    }
    return result;
}

uint8_t parseSensorId(const std::string& section) {
    constexpr const char* kPrefix = "lidar_";
    if (section.compare(0, 6, kPrefix) != 0 || section.size() == 6) {
        throw std::runtime_error("unknown section: [" + section + "]");
    }
    const std::string value = section.substr(6);
    for (const char c : value) {
        if (c < '0' || c > '9') {
            throw std::runtime_error("bad sensor ID in section [" + section + "]");
        }
    }
    const unsigned long id = std::stoul(value);
    if (id > 255) throw std::runtime_error("sensor ID must be in 0..255");
    return static_cast<uint8_t>(id);
}

// Wrap to (-pi, pi]. Map -pi to +pi so both representations behave identically.
double normalizeAngle(double angle) {
    double result = std::remainder(angle, 2.0 * kPi);
    if (result <= -kPi) result += 2.0 * kPi;
    return result;
}

std::vector<LidarBearingMask> parseMasks(const std::string& text) {
    std::vector<LidarBearingMask> ranges;
    if (trim(text).empty()) return ranges; // No physical occlusions defined yet.

    std::size_t from = 0;
    while (from < text.size()) {
        const auto comma = text.find(',', from);
        const std::string token = trim(text.substr(from, comma == std::string::npos
                                                            ? std::string::npos : comma - from));
        const auto colon = token.find(':');
        if (token.empty() || colon == std::string::npos ||
            token.find(':', colon + 1) != std::string::npos) {
            throw std::runtime_error("blocked_bearings_deg expects start:end pairs separated by commas");
        }
        const double start = parseDouble(token.substr(0, colon));
        const double end = parseDouble(token.substr(colon + 1));
        if (start < -180.0 || start > 180.0 || end < -180.0 || end > 180.0) {
            throw std::runtime_error("mask angles must be within -180..180 degrees");
        }
        if (start == end) {
            throw std::runtime_error("mask intervals must have nonzero width");
        }
        ranges.push_back({start * kRadiansPerDegree, end * kRadiansPerDegree});
        if (comma == std::string::npos) break;
        from = comma + 1;
        if (trim(text.substr(from)).empty()) {
            throw std::runtime_error("trailing comma in blocked_bearings_deg");
        }
    }
    return ranges;
}

void validate(const LidarMount& mount) {
    const auto& f = mount.footprint;
    if (!std::isfinite(mount.x_in) || !std::isfinite(mount.y_in) ||
        !std::isfinite(mount.psi_rad) || !std::isfinite(mount.mount_height_in) ||
        mount.mount_height_in < 0.0 || !std::isfinite(mount.r_min_in) ||
        !std::isfinite(mount.r_max_in) || mount.r_min_in < 0.0 ||
        mount.r_max_in <= mount.r_min_in) {
        throw std::invalid_argument("invalid lidar pose, height or distance limits");
    }
    for (const double v : {f.front_in, f.rear_in, f.left_in, f.right_in, f.margin_in}) {
        if (!std::isfinite(v) || v < 0.0) {
            throw std::invalid_argument("footprint dimensions and margin must be finite and >= 0");
        }
    }
    for (const auto& mask : mount.blocked_bearings) {
        if (!std::isfinite(mask.start_rad) || !std::isfinite(mask.end_rad) ||
            mask.start_rad < -kPi || mask.start_rad > kPi ||
            mask.end_rad < -kPi || mask.end_rad > kPi ||
            mask.start_rad == mask.end_rad) {
            throw std::invalid_argument("invalid sensor bearing mask");
        }
    }
}

struct MountInput {
    LidarMount mount;
    bool x = false, y = false, psi = false, minRange = false, maxRange = false, height = false;
    bool mask = false;
};

struct FootprintInput {
    LidarRobotFootprint footprint;
    bool front = false, rear = false, left = false, right = false, margin = false;
};

void setOnce(bool& provided, const std::string& key) {
    if (provided) throw std::runtime_error("duplicate key '" + key + "'");
    provided = true;
}

} // namespace

bool loadLidarMounts(const std::string& path, std::vector<LidarMount>& out) {
    std::ifstream input(path);
    if (!input) {
        std::cerr << "Cannot open lidar mount config: " << path << '\n';
        return false;
    }

    try {
        std::map<uint8_t, MountInput> mounts; // Keeps deterministic sensor order.
        FootprintInput robot;
        bool hasRobot = false;
        std::string section;
        std::string line;
        std::size_t lineNumber = 0;

        while (std::getline(input, line)) {
            ++lineNumber;
            try {
                const std::string statement = removeComment(line);
                if (statement.empty()) continue;

                if (statement.front() == '[') {
                    if (statement.back() != ']') throw std::runtime_error("malformed section header");
                    section = trim(statement.substr(1, statement.size() - 2));
                    if (section == "robot") {
                        if (hasRobot) throw std::runtime_error("duplicate [robot] section");
                        hasRobot = true;
                    } else {
                        const auto id = parseSensorId(section);
                        const auto inserted = mounts.emplace(id, MountInput{});
                        if (!inserted.second) throw std::runtime_error("duplicate sensor section");
                        inserted.first->second.mount.sensor_id = id;
                    }
                    continue;
                }

                const auto equal = statement.find('=');
                if (equal == std::string::npos || section.empty()) {
                    throw std::runtime_error("expected key=value inside a section");
                }
                const std::string key = trim(statement.substr(0, equal));
                const std::string value = trim(statement.substr(equal + 1));
                if (section == "robot") {
                    auto& r = robot;
                    if (key == "front_in") { setOnce(r.front, key); r.footprint.front_in = parseDouble(value); }
                    else if (key == "rear_in") { setOnce(r.rear, key); r.footprint.rear_in = parseDouble(value); }
                    else if (key == "left_in") { setOnce(r.left, key); r.footprint.left_in = parseDouble(value); }
                    else if (key == "right_in") { setOnce(r.right, key); r.footprint.right_in = parseDouble(value); }
                    else if (key == "margin_in") { setOnce(r.margin, key); r.footprint.margin_in = parseDouble(value); }
                    else throw std::runtime_error("unknown [robot] key: " + key);
                } else {
                    auto& r = mounts.at(parseSensorId(section));
                    if (key == "x_in") { setOnce(r.x, key); r.mount.x_in = parseDouble(value); }
                    else if (key == "y_in") { setOnce(r.y, key); r.mount.y_in = parseDouble(value); }
                    else if (key == "psi_rad") { setOnce(r.psi, key); r.mount.psi_rad = parseDouble(value); }
                    else if (key == "r_min_in") { setOnce(r.minRange, key); r.mount.r_min_in = parseDouble(value); }
                    else if (key == "r_max_in") { setOnce(r.maxRange, key); r.mount.r_max_in = parseDouble(value); }
                    else if (key == "mount_height_in") { setOnce(r.height, key); r.mount.mount_height_in = parseDouble(value); }
                    else if (key == "blocked_bearings_deg") { setOnce(r.mask, key); r.mount.blocked_bearings = parseMasks(value); }
                    else throw std::runtime_error("unknown sensor key: " + key);
                }
            } catch (const std::exception& e) {
                throw std::runtime_error("line " + std::to_string(lineNumber) + ": " + e.what());
            }
        }

        if (!input.eof() && input.fail()) throw std::runtime_error("I/O error while reading file");
        if (!hasRobot || !robot.front || !robot.rear || !robot.left || !robot.right || !robot.margin) {
            throw std::runtime_error("[robot] must define front_in, rear_in, left_in, right_in, margin_in");
        }
        if (mounts.empty()) throw std::runtime_error("no [lidar_ID] sections found");

        std::vector<LidarMount> loaded;
        loaded.reserve(mounts.size());
        for (auto& item : mounts) {
            auto& r = item.second;
            if (!r.x || !r.y || !r.psi || !r.minRange || !r.maxRange || !r.height) {
                throw std::runtime_error("[lidar_" + std::to_string(item.first) +
                                         "] missing x_in/y_in/psi_rad/r_min_in/r_max_in/mount_height_in");
            }
            r.mount.footprint = robot.footprint;
            validate(r.mount);
            loaded.push_back(std::move(r.mount));
        }
        out = std::move(loaded); // Only replace the output on complete success.
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Invalid lidar mount config '" << path << "': " << e.what() << '\n';
        return false;
    }
}

LidarTransform::LidarTransform(LidarMount mount) : m_mount(std::move(mount)) {
    validate(m_mount);
}

RobotScan LidarTransform::toRobotFrame(const Scan& scan) const {
    if (scan.sensor_id != m_mount.sensor_id) {
        throw std::invalid_argument("scan sensor_id does not match LidarTransform mount");
    }

    RobotScan result;
    result.sensor_id = scan.sensor_id;
    result.t_start_us = scan.t_start_us;
    result.t_end_us = scan.t_end_us;
    result.points.reserve(scan.points.size());

    const double c = std::cos(m_mount.psi_rad);
    const double s = std::sin(m_mount.psi_rad);

    for (const auto& point : scan.points) {
        // #209's bearing_rad is ALREADY CCW-positive. Do not negate it again.
        const double r = point.range_in;
        const double beta = point.bearing_rad;
        if (!std::isfinite(r) || !std::isfinite(beta) || r <= 0.0 ||
            r < m_mount.r_min_in || r > m_mount.r_max_in) {
            continue;
        }
        if (masked(beta)) continue; // Angular masks are in the sensor's frame.

        const double sensorX = r * std::cos(beta);
        const double sensorY = r * std::sin(beta);
        const double x = m_mount.x_in + c * sensorX - s * sensorY;
        const double y = m_mount.y_in + s * sensorX + c * sensorY;
        if (insideFootprint(x, y)) continue;

        result.points.push_back({x, y, point.stamp_us});
    }
    return result;
}

bool LidarTransform::masked(double bearing_rad) const {
    const double angle = normalizeAngle(bearing_rad);
    for (const auto& mask : m_mount.blocked_bearings) {
        // The -180/180 endpoint is physically identical. A -180:180 mask
        // intentionally blocks the whole circle.
        if ((mask.start_rad == -kPi && mask.end_rad == kPi) ||
            (mask.start_rad == kPi && mask.end_rad == -kPi)) return true;
        const double begin = normalizeAngle(mask.start_rad);
        const double end = normalizeAngle(mask.end_rad);
        if (begin <= end) {
            if (angle >= begin - kAngleEpsilon && angle <= end + kAngleEpsilon) return true;
        } else { // Wraps across +/-pi.
            if (angle >= begin - kAngleEpsilon || angle <= end + kAngleEpsilon) return true;
        }
    }
    return false;
}

bool LidarTransform::insideFootprint(double x_in, double y_in) const {
    const auto& f = m_mount.footprint;
    return x_in >= -f.rear_in - f.margin_in &&
           x_in <=  f.front_in + f.margin_in &&
           y_in >= -f.right_in - f.margin_in &&
           y_in <=  f.left_in + f.margin_in;
}

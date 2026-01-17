/**
 * @file types.cpp
 * @brief Implementation of core types
 *
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#include <aura/core/types.hpp>

#include <cmath>

namespace aura {

std::array<Point3D, 8> BoundingBox3D::corners() const noexcept {
    const Float half_l = length / 2.0;
    const Float half_w = width / 2.0;
    const Float half_h = height / 2.0;

    // Local corner points (before rotation)
    std::array<Point3D, 8> local_corners = {{{-half_l, -half_w, -half_h},
                                             {half_l, -half_w, -half_h},
                                             {half_l, half_w, -half_h},
                                             {-half_l, half_w, -half_h},
                                             {-half_l, -half_w, half_h},
                                             {half_l, -half_w, half_h},
                                             {half_l, half_w, half_h},
                                             {-half_l, half_w, half_h}}};

    // Apply yaw rotation and translation
    const Float cos_yaw = std::cos(yaw);
    const Float sin_yaw = std::sin(yaw);

    std::array<Point3D, 8> world_corners;
    for (std::size_t i = 0; i < 8; ++i) {
        const auto& local = local_corners[i];
        world_corners[i] = {center.x + cos_yaw * local.x - sin_yaw * local.y,
                            center.y + sin_yaw * local.x + cos_yaw * local.y, center.z + local.z};
    }

    return world_corners;
}

bool BoundingBox3D::contains(const Point3D& point) const noexcept {
    // Transform point to local frame
    const Float cos_yaw = std::cos(-yaw);
    const Float sin_yaw = std::sin(-yaw);

    const Float dx = point.x - center.x;
    const Float dy = point.y - center.y;
    const Float dz = point.z - center.z;

    const Float local_x = cos_yaw * dx - sin_yaw * dy;
    const Float local_y = sin_yaw * dx + cos_yaw * dy;

    const Float half_l = length / 2.0;
    const Float half_w = width / 2.0;
    const Float half_h = height / 2.0;

    return std::abs(local_x) <= half_l && std::abs(local_y) <= half_w && std::abs(dz) <= half_h;
}

}  // namespace aura

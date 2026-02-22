/**
 * @file data_association.hpp
 * @brief Data association algorithms for multi-target tracking
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <limits>
#include <set>
#include <vector>

#include <stddef.h>

namespace aura {
namespace fusion {

struct Association {
    int track_idx;
    int measurement_idx;
};

/**
 * @brief Global Nearest Neighbor (GNN) matching using a greedy approach
 *
 * @param distance_matrix N x M matrix of distances between Tracks (N) and Measurements (M)
 * @param threshold Maximum allowable distance for an association
 * @return std::vector<Association> List of associations
 */
inline std::vector<Association> globalNearestNeighborGreedy(
    const std::vector<std::vector<double>>& distance_matrix, double threshold) {
    std::vector<Association> associations;
    if (distance_matrix.empty() || distance_matrix[0].empty()) {
        return associations;
    }

    size_t num_tracks = distance_matrix.size();
    size_t num_meas = distance_matrix[0].size();

    std::set<size_t> assigned_tracks;
    std::set<size_t> assigned_meas;

    while (true) {
        double min_dist = std::numeric_limits<double>::max();
        int best_track = -1;
        int best_meas = -1;

        for (size_t t = 0; t < num_tracks; ++t) {
            if (assigned_tracks.find(t) != assigned_tracks.end())
                continue;
            for (size_t m = 0; m < num_meas; ++m) {
                if (assigned_meas.find(m) != assigned_meas.end())
                    continue;

                if (distance_matrix[t][m] < min_dist) {
                    min_dist = distance_matrix[t][m];
                    best_track = static_cast<int>(t);
                    best_meas = static_cast<int>(m);
                }
            }
        }

        if (best_track == -1 || min_dist > threshold) {
            break;
        }

        associations.push_back({best_track, best_meas});
        assigned_tracks.insert(static_cast<size_t>(best_track));
        assigned_meas.insert(static_cast<size_t>(best_meas));
    }

    return associations;
}

}  // namespace fusion
}  // namespace aura

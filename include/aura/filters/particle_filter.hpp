/**
 * @file particle_filter.hpp
 * @brief Particle Filter implementation for non-linear, non-Gaussian systems
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <aura/math/matrix.hpp>

#include <array>
#include <functional>
#include <numeric>
#include <random>

namespace aura {
namespace filters {

/**
 * @brief Particle Filter implementation
 *
 * @tparam StateDim Dimension of the state vector
 * @tparam MeasureDim Dimension of the measurement vector
 * @tparam NumParticles Number of particles
 * @tparam T Numeric type (default double)
 */
template <size_t StateDim, size_t MeasureDim, size_t NumParticles, typename T = double>
class ParticleFilter {
public:
    using StateVector = math::Vector<T, StateDim>;
    using StateMatrix = math::Matrix<T, StateDim, StateDim>;
    using MeasureVector = math::Vector<T, MeasureDim>;
    using MeasureMatrix = math::Matrix<T, MeasureDim, MeasureDim>;
    using ProcessNoiseMatrix = math::Matrix<T, StateDim, StateDim>;
    using MeasureNoiseMatrix = math::Matrix<T, MeasureDim, MeasureDim>;

    struct Particle {
        StateVector state;
        T weight;
    };

    ParticleFilter() : gen_(std::random_device{}()) {
        for (size_t i = 0; i < NumParticles; ++i) {
            particles_[i].state = StateVector::Zero();
            particles_[i].weight = 1.0 / NumParticles;
        }
    }

    void init(const StateVector& initial_state, const StateMatrix& initial_covariance) {
        // Sample particles from initial gaussian distribution
        // For simplicity, we assume uncorrelated dimensions if covariance is diagonal
        // Alternatively we can use cholesky for full covariance
        StateMatrix l_mat = initial_covariance.cholesky();
        std::normal_distribution<T> norm(0.0, 1.0);

        for (size_t i = 0; i < NumParticles; ++i) {
            StateVector random_vec;
            for (size_t j = 0; j < StateDim; ++j) {
                random_vec(j, 0) = norm(gen_);
            }
            particles_[i].state = initial_state + (l_mat * random_vec);
            particles_[i].weight = 1.0 / NumParticles;
        }
    }

    void predict(
        const std::function<StateVector(const StateVector&, std::mt19937&)>& f_with_noise) {
        for (size_t i = 0; i < NumParticles; ++i) {
            particles_[i].state = f_with_noise(particles_[i].state, gen_);
        }
    }

    void update(const MeasureVector& z,
                const std::function<T(const MeasureVector&, const StateVector&)>& weight_function) {
        T weight_sum = 0.0;
        for (size_t i = 0; i < NumParticles; ++i) {
            T w = weight_function(z, particles_[i].state);
            particles_[i].weight *= w;
            weight_sum += particles_[i].weight;
        }

        // Normalize weights
        if (weight_sum > 0.0) {
            for (size_t i = 0; i < NumParticles; ++i) {
                particles_[i].weight /= weight_sum;
            }
        } else {
            // all weights zero, reinitialize uniform
            for (size_t i = 0; i < NumParticles; ++i) {
                particles_[i].weight = 1.0 / NumParticles;
            }
        }
    }

    void resample() {
        // Systematic resampling
        std::array<Particle, NumParticles> new_particles;
        std::uniform_real_distribution<T> unif(0.0, 1.0 / NumParticles);
        T r = unif(gen_);
        T c = particles_[0].weight;
        size_t i = 0;

        for (size_t m = 0; m < NumParticles; ++m) {
            T u = r + m * (1.0 / NumParticles);
            while (u > c && i < NumParticles - 1) {
                i++;
                c += particles_[i].weight;
            }
            new_particles[m] = particles_[i];
            new_particles[m].weight = 1.0 / NumParticles;
        }

        particles_ = new_particles;
    }

    [[nodiscard]] StateVector getState() const {
        StateVector mean = StateVector::Zero();
        for (size_t i = 0; i < NumParticles; ++i) {
            for (size_t j = 0; j < StateDim; ++j) {
                mean(j, 0) += particles_[i].weight * particles_[i].state(j, 0);
            }
        }
        return mean;
    }

    [[nodiscard]] const std::array<Particle, NumParticles>& getParticles() const {
        return particles_;
    }

private:
    std::array<Particle, NumParticles> particles_;
    std::mt19937 gen_;
};

}  // namespace filters
}  // namespace aura

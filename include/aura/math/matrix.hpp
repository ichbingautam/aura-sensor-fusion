/**
 * @file matrix.hpp
 * @brief Lightweight compile-time matrix class for sensor fusion
 * @copyright Copyright (c) 2026 AURA Contributors
 * @license MIT License
 */

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <type_traits>

namespace aura {
namespace math {

/**
 * @brief A simple compile-time sized Matrix class
 *
 * @tparam T Element type
 * @tparam Rows Number of rows
 * @tparam Cols Number of columns
 */
template <typename T, size_t Rows, size_t Cols>
class Matrix {
public:
    using ValueType = T;
    static constexpr size_t RowCount = Rows;
    static constexpr size_t ColCount = Cols;

    // Default constructor (zero initialization)
    constexpr Matrix() : data_{} {}

    // Initializer list constructor
    constexpr Matrix(std::initializer_list<T> list) {
        if (list.size() != Rows * Cols) {
            throw std::invalid_argument("Initializer list size does not match matrix dimensions");
        }
        std::copy(list.begin(), list.end(), data_.begin());
    }

    // Accessors
    constexpr T& operator()(size_t r, size_t c) { return data_[r * Cols + c]; }

    constexpr const T& operator()(size_t r, size_t c) const { return data_[r * Cols + c]; }

    // Static factories
    static Matrix Zero() { return Matrix(); }

    static Matrix Identity() {
        static_assert(Rows == Cols, "Identity matrix must be square");
        Matrix m;
        for (size_t i = 0; i < Rows; ++i) {
            m(i, i) = static_cast<T>(1);
        }
        return m;
    }

    // Operations
    Matrix<T, Cols, Rows> transpose() const {
        Matrix<T, Cols, Rows> result;
        for (size_t r = 0; r < Rows; ++r) {
            for (size_t c = 0; c < Cols; ++c) {
                result(c, r) = (*this)(r, c);
            }
        }
        return result;
    }

    // Matrix addition
    Matrix operator+(const Matrix& other) const {
        Matrix result;
        for (size_t i = 0; i < Rows * Cols; ++i) {
            result.data_[i] = data_[i] + other.data_[i];
        }
        return result;
    }

    // Matrix subtraction
    Matrix operator-(const Matrix& other) const {
        Matrix result;
        for (size_t i = 0; i < Rows * Cols; ++i) {
            result.data_[i] = data_[i] - other.data_[i];
        }
        return result;
    }

    // Matrix multiplication
    template <size_t OtherCols>
    Matrix<T, Rows, OtherCols> operator*(const Matrix<T, Cols, OtherCols>& other) const {
        Matrix<T, Rows, OtherCols> result;
        for (size_t r = 0; r < Rows; ++r) {
            for (size_t c = 0; c < OtherCols; ++c) {
                T sum = 0;
                for (size_t k = 0; k < Cols; ++k) {
                    sum += (*this)(r, k) * other(k, c);
                }
                result(r, c) = sum;
            }
        }
        return result;
    }

    // Scalar multiplication
    Matrix operator*(T scalar) const {
        Matrix result;
        for (size_t i = 0; i < Rows * Cols; ++i) {
            result.data_[i] = data_[i] * scalar;
        }
        return result;
    }

    // Matrix inversion (using Partial Pivoting LU Decomposition)
    // Only implemented for square matrices
    Matrix inverse() const {
        static_assert(Rows == Cols, "Matrix inversion requires square matrix");

        // augmented matrix [A | I]
        Matrix<T, Rows, Cols * 2> aug;
        for (size_t i = 0; i < Rows; ++i) {
            for (size_t j = 0; j < Cols; ++j) {
                aug(i, j) = (*this)(i, j);
                aug(i, j + Cols) = (i == j) ? static_cast<T>(1) : static_cast<T>(0);
            }
        }

        // Gaussian elimination
        for (size_t i = 0; i < Rows; ++i) {
            // Find pivot
            size_t pivot = i;
            for (size_t j = i + 1; j < Rows; ++j) {
                if (std::abs(aug(j, i)) > std::abs(aug(pivot, i))) {
                    pivot = j;
                }
            }

            // Swap rows
            if (std::abs(aug(pivot, i)) < 1e-9) {
                throw std::runtime_error("Matrix is singular");
            }
            if (pivot != i) {
                for (size_t j = 0; j < 2 * Cols; ++j) {
                    std::swap(aug(i, j), aug(pivot, j));
                }
            }

            // Scale row
            T div = aug(i, i);
            for (size_t j = i; j < 2 * Cols; ++j) {
                aug(i, j) /= div;
            }

            // Eliminate other rows
            for (size_t k = 0; k < Rows; ++k) {
                if (k != i) {
                    T factor = aug(k, i);
                    for (size_t j = i; j < 2 * Cols; ++j) {
                        aug(k, j) -= factor * aug(i, j);
                    }
                }
            }
        }

        // Extract inverse
        Matrix inv;
        for (size_t i = 0; i < Rows; ++i) {
            for (size_t j = 0; j < Cols; ++j) {
                inv(i, j) = aug(i, j + Cols);
            }
        }
        return inv;
    }

    // Print
    friend std::ostream& operator<<(std::ostream& os, const Matrix& m) {
        os << "[";
        for (size_t r = 0; r < Rows; ++r) {
            if (r > 0)
                os << "; ";
            for (size_t c = 0; c < Cols; ++c) {
                if (c > 0)
                    os << ", ";
                os << m(r, c);
            }
        }
        os << "]";
        return os;
    }

private:
    std::array<T, Rows * Cols> data_;
};

// Vector alias
template <typename T, size_t Rows>
using Vector = Matrix<T, Rows, 1>;

}  // namespace math
}  // namespace aura

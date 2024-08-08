#pragma once

#include <array>
#include <cmath>
#include <iostream>

/**
 * Template functions to perform standard vector operations (+, -, norm, distance, scalar multiplication) on
 * std::array's.
 */

/**
 * Returns result of vector addition.
 */
template <size_t N>
std::array<double, N> vector_add(const std::array<double, N> &vec1, const std::array<double, N> &vec2) {
    std::array<double, N> res;
    for (size_t i = 0; i < N; i++) {
        res[i] = vec1[i] + vec2[i];
    }
    return res;
}

/**
 * Returns result of vector difference.
 */
template <size_t N>
std::array<double, N> vector_diff(const std::array<double, N> &vec1, const std::array<double, N> &vec2) {
    std::array<double, N> res;
    for (size_t i = 0; i < N; i++) {
        res[i] = vec1[i] - vec2[i];
    }
    return res;
}

/**
 * Returns the Euclidean norm of a vector.
 */
template <size_t N> double vector_norm(const std::array<double, N> &vec) {
    double norm = 0.0;
    for (size_t i = 0; i < N; i++) {
        norm += vec[i] * vec[i];
    }
    return std::sqrt(norm);
}

/**
 * Returns Euclidean distance between vectors.
 */
template <size_t N> double vector_distance(const std::array<double, N> &vec1, const std::array<double, N> &vec2) {
    std::array<double, N> diff = vector_diff(vec1, vec2);
    return vector_norm(diff);
}

/**
 * Returns the product of a vector and scalar.
 */
template <size_t N> std::array<double, N> vector_mult(double scalar, const std::array<double, N> &vec) {
    std::array<double, N> res;
    for (size_t i = 0; i < N; i++) {
        res[i] = scalar * vec[i];
    }
    return res;
}

/**
 * Prints elements of a vector
 */
template <size_t N> void vector_print(const std::array<double, N> &vec) {
    std::cout << "[";
    for (size_t i = 0; i < (N - 1); i++) {
        std::cout << vec[i] << ", ";
    }
    std::cout << vec[N - 1] << "]" << std::endl;
}

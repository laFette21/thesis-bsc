#ifndef UTILS_H
#define UTILS_H

#include <ceres/ceres.h>

/**
 * @brief Calculate the rotation matrix of a given angle.
 * 
 * @tparam T
 * @param theta
 * @return Eigen::Matrix2<T>
 */
template <typename T>
Eigen::Matrix2<T> RotationMatrix2D(const T& theta)
{
    Eigen::Matrix2<T> result;

    const T cos_theta = ceres::cos(theta);
    const T sin_theta = ceres::sin(theta);

    result << cos_theta, -sin_theta, sin_theta, cos_theta;

    return result;
}

/**
 * @brief Struct that represents a LandmarkErrorFunction functor used in the optimization.
 * The behavior is described in the thesis.
 * 
 */
struct LandmarkErrorFunction
{
    template <typename T>
    bool operator()(const T* const pose, const T* const landmark, const T* const meas, T* residual) const
    {
        Eigen::Matrix2<T> rotation = RotationMatrix2D<T>(pose[2]);
        T lm_meas_x = meas[0] * ceres::cos(meas[1]);
        T lm_meas_y = meas[0] * ceres::sin(meas[1]);

        Eigen::Vector2<T> temp;
        temp(0) = landmark[0] - pose[0];
        temp(1) = landmark[1] - pose[1];

        Eigen::Map<Eigen::Vector2<T>> residual_map(residual);
        residual_map = rotation.transpose() * temp;

        residual_map(0) -= lm_meas_x;
        residual_map(1) -= lm_meas_y;

        return true;
    }
};

/**
 * @brief Struct that represents a PoseErrorFunction functor used in the optimization.
 * The behavior is described in the thesis.
 * 
 */
struct PoseErrorFunction
{
    template <typename T>
    bool operator()(const T* const prev, const T* const curr, const T* const meas, T* residual) const
    {
        T meas_x_global = meas[0] * ceres::cos(prev[2] + meas[1]);
        T meas_y_global = meas[0] * ceres::sin(prev[2] + meas[1]);

        residual[0] = (curr[0] - prev[0]) - meas_x_global;
        residual[1] = (curr[1] - prev[1]) - meas_y_global;
        residual[2] = (curr[2] - prev[2]) - meas[1] * 2.0;

        return true;
    }
};

#endif // UTILS_H

#ifndef UTILS_H
#define UTILS_H

#include <ceres/ceres.h>

template <typename T>
inline T NormalizeAngle(const T& theta)
{
    const T two_pi(2.0 * M_PI);

    return theta - two_pi * ceres::floor((theta + T(M_PI)) / two_pi);
}

template <typename T>
Eigen::Matrix2<T> RotationMatrix2D(const T& theta)
{
    Eigen::Matrix2<T> result;

    const T cos_theta = ceres::cos(theta);
    const T sin_theta = ceres::sin(theta);

    result << cos_theta, -sin_theta, sin_theta, cos_theta;

    return result;
}
/*
template <typename T>
Eigen::Vector3<T> t2v(const Eigen::Matrix3<T>& transformation)
{
    Eigen::Vector3<T> result;

    result[0] = transformation(0, 2);
    result[1] = transformation(1, 2);
    result[2] = ceres::atan2(transformation(1, 0), transformation(0, 0));

    return result;
}

template <typename T>
Eigen::Matrix3<T> v2t(const Eigen::Vector3<T>& vector)
{
    Eigen::Matrix3<T> result = Eigen::Matrix3<T>::Zero();

    result.template topLeftCorner<2, 2>() = RotationMatrix2D<T>(vector[2]);
    result.template topRightCorner<2, 1>() = vector.template head<2>();
    result(2, 2) = T(1);

    return result;
}
*/
struct LandmarkErrorFunction
{
    template <typename T>
    bool operator()(const T* const pose, const T* const landmark, const T* const measurement, T* residual) const
    {
        Eigen::Matrix2<T> rotation = RotationMatrix2D<T>(pose[2]);
        T lm_meas_x = measurement[0] * ceres::cos(measurement[1] + pose[2]); // TODO: - pose[2] -> {lm_meas_x, lm_meas_y} * rotation
        T lm_meas_y = measurement[0] * ceres::sin(measurement[1] + pose[2]);

        Eigen::Vector2<T> temp;
        temp(0) = landmark[0] - pose[0];
        temp(1) = landmark[1] - pose[1];

        temp = rotation.transpose() * temp;

        residual[0] = temp(0) - lm_meas_x;
        residual[1] = temp(1) - lm_meas_y;

        return true;
    }
};

struct PoseErrorFunction
{
    template <typename T>
    bool operator()(const T* const prev, const T* const curr, const T* const meas, T* residual) const
    {
        double ts = 0.05; // 50 ms
        T meas_x_global = meas[0] * ts * ceres::cos(prev[2] + meas[1] * ts * 0.5); // TODO: outsource the ts multiplication
        T meas_y_global = meas[0] * ts * ceres::sin(prev[2] + meas[1] * ts * 0.5);
        T meas_psi_global = meas[1] * ts;

        residual[0] = (curr[0] - prev[0]) - meas_x_global;
        residual[1] = (curr[1] - prev[1]) - meas_y_global;
        residual[2] = NormalizeAngle((curr[2] - prev[2]) - meas_psi_global);

        return true;
    }
};

#endif // UTILS_H

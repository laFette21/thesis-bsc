#ifndef UTILS_H
#define UTILS_H

#include <ceres/ceres.h>

template <typename T>
Eigen::Matrix2<T> RotationMatrix2D(T theta)
{
    Eigen::Matrix2<T> result;

    const T cos_theta = ceres::cos(theta);
    const T sin_theta = ceres::sin(theta);

    result << cos_theta, -sin_theta, sin_theta, cos_theta;

    return result;
}

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

struct LandmarkErrorFunction
{
    template <typename T>
    bool operator()(const T* const pose, const T* const landmark, const T* const measurement, T* residual) const
    {
        Eigen::Matrix2<T> rotation = RotationMatrix2D<T>(pose[2]);

        Eigen::Vector2<T> temp;
        temp(0) = landmark[0] - pose[0];
        temp(1) = landmark[1] - pose[1];

        temp = rotation.transpose() * temp;

        residual[0] = temp(0) - measurement[0];
        residual[1] = temp(1) - measurement[1];

        return true;
    }
};

struct PoseErrorFunction
{
    template <typename T>
    bool operator()(const T* const prev, const T* const curr, const T* const meas, T* residual) const
    {
        Eigen::Matrix3<T> prev_pose_trans = v2t<T>(Eigen::Vector3<T>(prev[0], prev[1], prev[2]));
        Eigen::Matrix3<T> curr_pose_trans = v2t<T>(Eigen::Vector3<T>(curr[0], curr[1], curr[2]));
        Eigen::Matrix3<T> measurement_trans = v2t<T>(Eigen::Vector3<T>(meas[0], meas[1], meas[2]));
        Eigen::Vector3<T> vec = t2v<T>((curr_pose_trans.inverse() * prev_pose_trans) * measurement_trans);

        residual[0] = vec[0];
        residual[1] = vec[1];
        residual[2] = vec[2];

        return true;
    }
};

#endif // UTILS_H

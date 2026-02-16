#include "detection_mocker/transforms.hpp"
#include <cmath>

namespace detection_mocker
{

    Eigen::Quaterniond Transforms::rpyToQuaternion(const Eigen::Vector3d &rpy)
    {
        // Roll-Pitch-Yaw to quaternion (ZYX convention)
        double roll = rpy(0);
        double pitch = rpy(1);
        double yaw = rpy(2);

        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        return yawAngle * pitchAngle * rollAngle;
    }

    Eigen::Vector3d Transforms::worldToCameraFrame(
        const Eigen::Vector3d &point_world,
        const Eigen::Vector3d &robot_position,
        const Eigen::Quaterniond &robot_orientation,
        const Eigen::Vector3d &camera_offset,
        const Eigen::Vector3d &camera_rotation_rpy)
    {
        const Eigen::Matrix3d camera_rotation = rpyToQuaternion(camera_rotation_rpy).toRotationMatrix();

        // Transform point from world to robot frame
        // p_robot = R_robot^T * (p_world - t_robot)
        const Eigen::Vector3d point_robot = robot_orientation.inverse() * (point_world - robot_position);

        // Transform point from robot to camera frame
        // p_camera = R_camera^T * (p_robot - t_camera)
        return camera_rotation.transpose() * (point_robot - camera_offset);
    }

} // namespace detection_mocker

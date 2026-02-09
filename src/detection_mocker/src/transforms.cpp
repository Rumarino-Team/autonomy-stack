#include "detection_mocker/transforms.hpp"
#include <cmath>

namespace detection_mocker
{

    Eigen::Matrix3d Transforms::rpyToRotationMatrix(const Eigen::Vector3d &rpy)
    {
        // Roll-Pitch-Yaw to rotation matrix (ZYX convention)
        double roll = rpy(0);
        double pitch = rpy(1);
        double yaw = rpy(2);

        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        return q.matrix();
    }

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

    Eigen::Vector3d Transforms::worldToRobotFrame(
        const Eigen::Vector3d &point_world,
        const Eigen::Vector3d &robot_position,
        const Eigen::Quaterniond &robot_orientation)
    {
        // Transform point from world to robot frame
        // p_robot = R_robot^T * (p_world - t_robot)
        Eigen::Vector3d translated = point_world - robot_position;
        return robot_orientation.inverse() * translated;
    }

    Eigen::Vector3d Transforms::robotToCameraFrame(
        const Eigen::Vector3d &point_robot,
        const Eigen::Vector3d &camera_offset,
        const Eigen::Matrix3d &camera_rotation)
    {
        // Transform point from robot to camera frame
        // p_camera = R_camera^T * (p_robot - t_camera)
        Eigen::Vector3d translated = point_robot - camera_offset;
        return camera_rotation.transpose() * translated;
    }

    Eigen::Vector3d Transforms::worldToCameraFrame(
        const Eigen::Vector3d &point_world,
        const Eigen::Vector3d &robot_position,
        const Eigen::Quaterniond &robot_orientation,
        const Eigen::Vector3d &camera_offset,
        const Eigen::Matrix3d &camera_rotation)
    {
        // Combined transformation: world -> robot -> camera
        Eigen::Vector3d point_robot = worldToRobotFrame(point_world, robot_position, robot_orientation);
        return robotToCameraFrame(point_robot, camera_offset, camera_rotation);
    }

} // namespace detection_mocker

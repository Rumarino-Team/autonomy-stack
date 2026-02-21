#ifndef DETECTION_MOCKER__TRANSFORMS_HPP_
#define DETECTION_MOCKER__TRANSFORMS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace detection_mocker
{

    /**
     * @brief Coordinate transformation utilities
     */
    class Transforms
    {
    public:
        /**
         * @brief Convert roll-pitch-yaw angles to quaternion
         * @param rpy Roll, pitch, yaw in radians
         * @return Quaternion
         */
        static Eigen::Quaterniond rpyToQuaternion(const Eigen::Vector3d &rpy);

        /**
         * @brief Transform point from world frame directly to camera frame
         * @param point_world Point in world coordinates
         * @param robot_position Robot position in world frame
         * @param robot_orientation Robot orientation (quaternion)
         * @param camera_offset Camera position offset from robot base
            * @param camera_rotation_rpy Camera rotation as roll-pitch-yaw (radians)
         * @return Point in camera frame
         */
        static Eigen::Vector3d worldToCameraFrame(
            const Eigen::Vector3d &point_world,
            const Eigen::Vector3d &robot_position,
            const Eigen::Quaterniond &robot_orientation,
            const Eigen::Vector3d &camera_offset,
                const Eigen::Vector3d &camera_rotation_rpy);
    };

} // namespace detection_mocker

#endif // DETECTION_MOCKER__TRANSFORMS_HPP_

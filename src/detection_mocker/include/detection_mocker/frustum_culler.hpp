#ifndef DETECTION_MOCKER__FRUSTUM_CULLER_HPP_
#define DETECTION_MOCKER__FRUSTUM_CULLER_HPP_

#include <Eigen/Dense>
#include <sensor_msgs/msg/camera_info.hpp>

namespace detection_mocker
{

    /**
     * @brief Camera frustum culling for visibility determination
     */
    class FrustumCuller
    {
    public:
        /**
         * @brief Constructor
         * @param min_distance Minimum detection distance (meters)
         * @param max_distance Maximum detection distance (meters)
         */
        FrustumCuller(double min_distance, double max_distance);

        /**
         * @brief Check if a point in camera frame is visible
         * @param point_camera Point coordinates in camera frame
         * @param horizontal_fov Horizontal field of view (radians)
         * @param vertical_fov Vertical field of view (radians)
         * @return true if point is within frustum
         */
        bool isVisible(const Eigen::Vector3d &point_camera,
                       double horizontal_fov,
                       double vertical_fov) const;

        /**
         * @brief Calculate horizontal FOV from camera intrinsics
         * @param camera_info Camera info message
         * @return Horizontal FOV in radians
         */
        static double calculateHorizontalFOV(const sensor_msgs::msg::CameraInfo &camera_info);

    private:
        double min_distance_;
        double max_distance_;
    };

} // namespace detection_mocker

#endif // DETECTION_MOCKER__FRUSTUM_CULLER_HPP_

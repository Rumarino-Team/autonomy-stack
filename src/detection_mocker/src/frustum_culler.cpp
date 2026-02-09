#include "detection_mocker/frustum_culler.hpp"
#include <cmath>

namespace detection_mocker
{

    FrustumCuller::FrustumCuller(double min_distance, double max_distance)
        : min_distance_(min_distance), max_distance_(max_distance)
    {
    }

    bool FrustumCuller::isVisible(const Eigen::Vector3d &point_camera,
                                  double horizontal_fov,
                                  double vertical_fov) const
    {
        // Camera frame: +X right, +Y down, +Z forward
        double x = point_camera.x();
        double y = point_camera.y();
        double z = point_camera.z();

        // Check if point is in front of camera (within distance range)
        if (z <= min_distance_ || z >= max_distance_)
        {
            return false;
        }

        // Check horizontal field of view
        double tan_half_hfov = std::tan(horizontal_fov / 2.0);
        if (std::abs(x) > z * tan_half_hfov)
        {
            return false;
        }

        // Check vertical field of view
        double tan_half_vfov = std::tan(vertical_fov / 2.0);
        if (std::abs(y) > z * tan_half_vfov)
        {
            return false;
        }

        return true;
    }

    double FrustumCuller::calculateHorizontalFOV(const sensor_msgs::msg::CameraInfo &camera_info)
    {
        // Calculate FOV from camera intrinsics
        // K[0] = fx (focal length in x)
        // K[2] = cx (principal point x)
        // FOV = 2 * atan(width / (2 * fx))
        double fx = camera_info.k[0];
        if (fx == 0.0)
        {
            // Uncalibrated camera, return default (60 degrees)
            return 60.0 * M_PI / 180.0;
        }
        return 2.0 * std::atan(camera_info.width / (2.0 * fx));
    }

    double FrustumCuller::calculateVerticalFOV(const sensor_msgs::msg::CameraInfo &camera_info)
    {
        // Calculate FOV from camera intrinsics
        // K[4] = fy (focal length in y)
        // K[5] = cy (principal point y)
        // FOV = 2 * atan(height / (2 * fy))
        double fy = camera_info.k[4];
        if (fy == 0.0)
        {
            // Uncalibrated camera, calculate from aspect ratio and horizontal FOV
            double hfov = calculateHorizontalFOV(camera_info);
            double aspect_ratio = static_cast<double>(camera_info.height) / camera_info.width;
            return 2.0 * std::atan(std::tan(hfov / 2.0) * aspect_ratio);
        }
        return 2.0 * std::atan(camera_info.height / (2.0 * fy));
    }

} // namespace detection_mocker

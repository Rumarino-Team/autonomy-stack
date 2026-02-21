#include "detection_mocker/frustum_culler.hpp"
#include <cmath>

namespace detection_mocker
{

    FrustumCuller::FrustumCuller()
        : FrustumCuller(1.0, 50.0)
    {
    }

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

} // namespace detection_mocker

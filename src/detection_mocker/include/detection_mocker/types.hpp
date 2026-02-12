#ifndef DETECTION_MOCKER__TYPES_HPP_
#define DETECTION_MOCKER__TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <string>

namespace detection_mocker
{

    /**
     * @brief Supported object types in scene files
     */
    enum class ObjectType
    {
        BOX,
        CYLINDER,
        PLANE,
        MODEL // Custom mesh objects
    };

    /**
     * @brief Static object parsed from .scn file
     */
    struct StaticObject
    {
        std::string name;
        ObjectType type;
        Eigen::Vector3d position;
        Eigen::Vector3d rotation_rpy;  // Roll, pitch, yaw
        Eigen::Vector3d dimensions;
        std::string mesh_filename;     // For MODEL type only
    };

    /**
     * @brief Camera configuration from robot .scn file
     */
    struct CameraConfig
    {
        Eigen::Vector3d offset;          // Camera offset from robot origin
        Eigen::Vector3d rotation_rpy;    // Roll, pitch, yaw
        int resolution_x = 0;            // Image width in pixels
        int resolution_y = 0;            // Image height in pixels
        double horizontal_fov_rad = 0.0; // Horizontal FOV in radians
    };

} // namespace detection_mocker

#endif // DETECTION_MOCKER__TYPES_HPP_

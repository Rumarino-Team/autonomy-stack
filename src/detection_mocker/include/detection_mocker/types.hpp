#ifndef DETECTION_MOCKER__TYPES_HPP_
#define DETECTION_MOCKER__TYPES_HPP_

#include <Eigen/Dense>
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
        Eigen::Vector3d position;     // World position (x, y, z)
        Eigen::Vector3d rotation_rpy; // Rotation in roll-pitch-yaw (radians)
        Eigen::Vector3d dimensions;   // Type-dependent:
                                      // - BOX: (width, depth, height)
                                      // - CYLINDER: (radius, height, 0)
                                      // - MODEL: (width, depth, height) from lookup
                                      // - PLANE: unused
        std::string mesh_filename;    // For MODEL type only
    };

    /**
     * @brief Camera configuration from robot .scn file
     */
    struct CameraConfig
    {
        Eigen::Vector3d offset;       // Position offset from robot base (x, y, z)
        Eigen::Vector3d rotation_rpy; // Rotation from robot base in roll-pitch-yaw (radians)
    };

} // namespace detection_mocker

#endif // DETECTION_MOCKER__TYPES_HPP_

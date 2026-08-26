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
     * @brief The Classifer Class Types from Vision.
     */
    // Must match mission_executor ObjectCls (src/mission_executor/src/main.rs).
    enum class ClassType
    {
        CUBE = 0,
        RECTANGLE = 1,
        GATE = 2,
        SHARK = 3,
        OTHER = 4,
        SWORD_FISH = 5,
    };


    /**
     * @brief Static object parsed from .scn file
     */
    struct StaticObject
    {
        std::string name;
        ObjectType type;
        ClassType cls;
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

    /**
     * @brief Map bounds computed from static objects
     */
    struct MapBounds
    {
        Eigen::Vector3d center;
        Eigen::Vector3d size;
    };

} // namespace detection_mocker

#endif // DETECTION_MOCKER__TYPES_HPP_

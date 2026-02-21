#ifndef DETECTION_MOCKER__XML_PARSER_HPP_
#define DETECTION_MOCKER__XML_PARSER_HPP_

#include "detection_mocker/types.hpp"
#include "tinyxml2.h"
#include <vector>
#include <string>

namespace detection_mocker
{

    /**
     * @brief Parser for Stonefish .scn XML files
     */
    class XMLParser
    {
    public:
        /**
         * @brief Set base path used to resolve relative mesh filenames
         * @param mesh_base_path Absolute base path (e.g., package data directory)
         */
        static void setMeshBasePath(const std::string &mesh_base_path);

        /**
         * @brief Parse static objects from environment scene file
         * @param scn_file_path Path to the .scn file
         * @return Vector of static objects
         * @throws std::runtime_error if file cannot be loaded or parsed
         */
        static std::vector<StaticObject> parseStaticObjects(const std::string& scn_file_path);

        /**
         * @brief Parse camera configuration from robot scene file
         * @param robot_scn_file_path Path to the robot .scn file
         * @return Camera configuration
         * @throws std::runtime_error if file cannot be loaded or camera not found
         */
        static CameraConfig parseCameraConfig(const std::string &robot_scn_file_path);

        /**
         * @brief Compute map bounds from static objects
         * @param objects Vector of static objects
         * @param padding_factor Padding multiplier (default 1.1 = 10% padding)
         * @return Map bounds
         */
        static MapBounds computeMapBounds(const std::vector<StaticObject> &objects, double padding_factor = 1.1);

    private:
        static std::string mesh_base_path_;

        /**
         * @brief Parse a single <static> XML element
         */
        static StaticObject parseStaticElement(tinyxml2::XMLElement *element);

        /**
         * @brief Parse dimensions based on object type
         */
        static Eigen::Vector3d parseDimensions(tinyxml2::XMLElement *element, ObjectType type);

        /**
         * @brief Parse world_transform attribute (xyz and rpy)
         */
        static void parseWorldTransform(tinyxml2::XMLElement *element,
                                        Eigen::Vector3d &position,
                                        Eigen::Vector3d &rotation);

        /**
         * @brief Get approximate bounding box dimensions for custom mesh
         * @param mesh_filename Mesh file path from XML
         * @return Estimated dimensions (x, y, z)
         */
        static Eigen::Vector3d getCustomMeshDimensions(const std::string &mesh_filename);

        /**
         * @brief Parse vector from space-separated string "x y z"
         */
        static Eigen::Vector3d parseVector3(const std::string &str);
    };

} // namespace detection_mocker

#endif // DETECTION_MOCKER__XML_PARSER_HPP_

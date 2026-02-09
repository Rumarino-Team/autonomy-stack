#include "detection_mocker/xml_parser.hpp"
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <map>

namespace detection_mocker
{

    std::vector<StaticObject> XMLParser::parseStaticObjects(const std::string &scn_file_path)
    {
        std::vector<StaticObject> objects;

        tinyxml2::XMLDocument doc;
        tinyxml2::XMLError result = doc.LoadFile(scn_file_path.c_str());

        if (result != tinyxml2::XML_SUCCESS)
        {
            throw std::runtime_error("Failed to load scene file: " + scn_file_path);
        }

        tinyxml2::XMLElement *root = doc.RootElement();
        if (!root)
        {
            throw std::runtime_error("No root element in scene file");
        }

        // Find all <static> elements
        for (tinyxml2::XMLElement *element = root->FirstChildElement("static");
             element != nullptr;
             element = element->NextSiblingElement("static"))
        {
            try
            {
                StaticObject obj = parseStaticElement(element);
                objects.push_back(obj);
            }
            catch (const std::exception &e)
            {
                std::cerr << "Warning: Failed to parse static object: " << e.what() << std::endl;
            }
        }

        std::cout << "Parsed " << objects.size() << " static objects from " << scn_file_path << std::endl;
        return objects;
    }

    CameraConfig XMLParser::parseCameraConfig(const std::string &robot_scn_file_path)
    {
        tinyxml2::XMLDocument doc;
        tinyxml2::XMLError result = doc.LoadFile(robot_scn_file_path.c_str());

        if (result != tinyxml2::XML_SUCCESS)
        {
            throw std::runtime_error("Failed to load robot scene file: " + robot_scn_file_path);
        }

        tinyxml2::XMLElement *root = doc.RootElement();
        if (!root)
        {
            throw std::runtime_error("No root element in robot scene file");
        }

        // Find <robot> element
        tinyxml2::XMLElement *robot = root->FirstChildElement("robot");
        if (!robot)
        {
            throw std::runtime_error("No <robot> element found in robot scene file");
        }

        // Find <sensor> element with type="camera"
        for (tinyxml2::XMLElement *sensor = robot->FirstChildElement("sensor");
             sensor != nullptr;
             sensor = sensor->NextSiblingElement("sensor"))
        {
            const char *type = nullptr;
            sensor->QueryStringAttribute("type", &type);

            if (type && std::string(type) == "camera")
            {
                CameraConfig config;

                // Find <origin> element
                tinyxml2::XMLElement *origin = sensor->FirstChildElement("origin");
                if (origin)
                {
                    const char *xyz_str = nullptr;
                    const char *rpy_str = nullptr;

                    origin->QueryStringAttribute("xyz", &xyz_str);
                    origin->QueryStringAttribute("rpy", &rpy_str);

                    if (xyz_str)
                    {
                        config.offset = parseVector3(std::string(xyz_str));
                    }
                    if (rpy_str)
                    {
                        config.rotation_rpy = parseVector3(std::string(rpy_str));
                    }

                    std::cout << "Camera config - Offset: (" << config.offset.transpose()
                              << "), RPY: (" << config.rotation_rpy.transpose() << ")" << std::endl;
                    return config;
                }
            }
        }

        throw std::runtime_error("Camera sensor not found in robot scene file");
    }

    StaticObject XMLParser::parseStaticElement(tinyxml2::XMLElement *element)
    {
        StaticObject obj;

        // Parse name
        const char *name = nullptr;
        element->QueryStringAttribute("name", &name);
        if (name)
        {
            obj.name = std::string(name);
        }

        // Parse type
        const char *type_str = nullptr;
        element->QueryStringAttribute("type", &type_str);
        if (!type_str)
        {
            throw std::runtime_error("Static object missing 'type' attribute");
        }

        std::string type = std::string(type_str);
        if (type == "box")
        {
            obj.type = ObjectType::BOX;
        }
        else if (type == "cylinder")
        {
            obj.type = ObjectType::CYLINDER;
        }
        else if (type == "plane")
        {
            obj.type = ObjectType::PLANE;
        }
        else if (type == "model")
        {
            obj.type = ObjectType::MODEL;
        }
        else
        {
            throw std::runtime_error("Unknown object type: " + type);
        }

        // Parse dimensions
        obj.dimensions = parseDimensions(element, obj.type);

        // Parse world transform
        parseWorldTransform(element, obj.position, obj.rotation_rpy);

        // For MODEL type, try to get mesh filename
        if (obj.type == ObjectType::MODEL)
        {
            tinyxml2::XMLElement *physical = element->FirstChildElement("physical");
            if (physical)
            {
                tinyxml2::XMLElement *mesh = physical->FirstChildElement("mesh");
                if (mesh)
                {
                    const char *filename = nullptr;
                    mesh->QueryStringAttribute("filename", &filename);
                    if (filename)
                    {
                        obj.mesh_filename = std::string(filename);
                        obj.dimensions = getCustomMeshDimensions(obj.mesh_filename);
                    }
                }
            }
        }

        return obj;
    }

    Eigen::Vector3d XMLParser::parseDimensions(tinyxml2::XMLElement *element, ObjectType type)
    {
        tinyxml2::XMLElement *dimensions = element->FirstChildElement("dimensions");
        if (!dimensions)
        {
            if (type == ObjectType::PLANE)
            {
                return Eigen::Vector3d::Zero(); // Planes don't have dimensions
            }
            throw std::runtime_error("Missing dimensions element");
        }

        const char *xyz_str = nullptr;
        const char *radius_str = nullptr;
        const char *height_str = nullptr;

        switch (type)
        {
        case ObjectType::BOX:
            dimensions->QueryStringAttribute("xyz", &xyz_str);
            if (xyz_str)
            {
                return parseVector3(std::string(xyz_str));
            }
            throw std::runtime_error("Box missing 'xyz' dimensions");

        case ObjectType::CYLINDER:
            dimensions->QueryStringAttribute("radius", &radius_str);
            dimensions->QueryStringAttribute("height", &height_str);
            if (radius_str && height_str)
            {
                return Eigen::Vector3d(
                    std::stod(radius_str),
                    std::stod(height_str),
                    0.0);
            }
            throw std::runtime_error("Cylinder missing 'radius' or 'height'");

        case ObjectType::PLANE:
            return Eigen::Vector3d::Zero();

        case ObjectType::MODEL:
            // Will be set from mesh lookup later
            return Eigen::Vector3d(0.5, 0.5, 0.5); // Default
        }

        return Eigen::Vector3d::Zero();
    }

    void XMLParser::parseWorldTransform(tinyxml2::XMLElement *element,
                                        Eigen::Vector3d &position,
                                        Eigen::Vector3d &rotation)
    {
        tinyxml2::XMLElement *transform = element->FirstChildElement("world_transform");
        if (!transform)
        {
            position = Eigen::Vector3d::Zero();
            rotation = Eigen::Vector3d::Zero();
            return;
        }

        const char *xyz_str = nullptr;
        const char *rpy_str = nullptr;

        transform->QueryStringAttribute("xyz", &xyz_str);
        transform->QueryStringAttribute("rpy", &rpy_str);

        if (xyz_str)
        {
            position = parseVector3(std::string(xyz_str));
        }
        if (rpy_str)
        {
            rotation = parseVector3(std::string(rpy_str));
        }
    }

    Eigen::Vector3d XMLParser::getCustomMeshDimensions(const std::string &mesh_filename)
    {
        // Lookup table for known meshes
        static const std::map<std::string, Eigen::Vector3d> mesh_dims = {
            {"models/hydrus_lowlegs.obj", Eigen::Vector3d(0.1, 0.75, 0.05)},
            {"hydrus_lowlegs.obj", Eigen::Vector3d(0.1, 0.75, 0.05)},
            // Add more known meshes here as needed
        };

        // Search for filename in lookup table
        for (const auto &[key, dims] : mesh_dims)
        {
            if (mesh_filename.find(key) != std::string::npos)
            {
                return dims;
            }
        }

        // Default fallback for unknown meshes
        std::cerr << "Warning: Unknown mesh '" << mesh_filename
                  << "', using default bounding box (0.5m cube)" << std::endl;
        return Eigen::Vector3d(0.5, 0.5, 0.5);
    }

    Eigen::Vector3d XMLParser::parseVector3(const std::string &str)
    {
        std::istringstream iss(str);
        double x, y, z;
        if (!(iss >> x >> y >> z))
        {
            throw std::runtime_error("Failed to parse vector3: " + str);
        }
        return Eigen::Vector3d(x, y, z);
    }

} // namespace detection_mocker

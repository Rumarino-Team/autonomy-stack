#include "detection_mocker/xml_parser.hpp"
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <map>
#include <cmath>
#include <fstream>
#include <limits>

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

                config.offset = Eigen::Vector3d::Zero();
                config.rotation_rpy = Eigen::Vector3d::Zero();

                tinyxml2::XMLElement *specs = sensor->FirstChildElement("specs");
                if (!specs)
                {
                    throw std::runtime_error("Camera <sensor> missing <specs> element");
                }

                int resolution_x = 0;
                int resolution_y = 0;
                double horizontal_fov_deg = 0.0;

                if (specs->QueryIntAttribute("resolution_x", &resolution_x) != tinyxml2::XML_SUCCESS ||
                    specs->QueryIntAttribute("resolution_y", &resolution_y) != tinyxml2::XML_SUCCESS ||
                    specs->QueryDoubleAttribute("horizontal_fov", &horizontal_fov_deg) != tinyxml2::XML_SUCCESS)
                {
                    throw std::runtime_error("Camera <specs> missing resolution_x, resolution_y, or horizontal_fov");
                }

                if (resolution_x <= 0 || resolution_y <= 0 || horizontal_fov_deg <= 0.0)
                {
                    throw std::runtime_error("Camera <specs> has invalid resolution or FOV values");
                }

                config.resolution_x = resolution_x;
                config.resolution_y = resolution_y;
                config.horizontal_fov_rad = horizontal_fov_deg * M_PI / 180.0;

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
        // GATE,
        // BOUY,
        // PATH,
        // BIND,
        // SHARK,
        // SWORDFISH,

        const char* cls_str = nullptr;
        element->QueryStringAttribute("cls", &cls_str);
        if(!cls_str)
        {
            // Default to PATH if cls attribute is missing
            obj.cls = ClassType::PATH;
        }
        else
        {
            // Make This A Function when needed refactor.
            std::string cls = std::string(cls_str);
            if( cls == "gate" )
            {
                obj.cls = ClassType::GATE;
            }
            else if( cls == "bouy")
            {
                obj.cls = ClassType::BOUY;
            }
            else if( cls == "path")
            {
                obj.cls = ClassType::PATH;
            }
            else if( cls == "bind")
            {
                obj.cls = ClassType::BIND;
            }
            else if( cls == "shark")
            {
                obj.cls = ClassType::SHARK;
            }
            else if(cls =="swordfish"){
                obj.cls = ClassType::SWORDFISH;
            }
            else
            {
                // Unknown class, default to PATH
                obj.cls = ClassType::PATH;
            }
        }

        // Parse dimensions
        Eigen::Vector3d dimensions = parseDimensions(element, obj.type);
        obj.dimensions = dimensions;

        // Parse world transform
        Eigen::Vector3d position, rotation_rpy;
        parseWorldTransform(element, position, rotation_rpy);
        obj.position = position;
        obj.rotation_rpy = rotation_rpy;

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
        static std::map<std::string, Eigen::Vector3d> mesh_dims_cache;

        auto cached = mesh_dims_cache.find(mesh_filename);
        if (cached != mesh_dims_cache.end())
        {
            return cached->second;
        }

        std::ifstream mesh_file(mesh_filename);
        if (!mesh_file.is_open())
        {
            std::cerr << "Warning: Failed to open mesh '" << mesh_filename
                      << "', using default bounding box (0.5m cube)" << std::endl;
            Eigen::Vector3d fallback(0.5, 0.5, 0.5);
            mesh_dims_cache.emplace(mesh_filename, fallback);
            return fallback;
        }

        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double min_z = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        double max_z = -std::numeric_limits<double>::infinity();

        std::string line;
        bool has_vertex = false;
        while (std::getline(mesh_file, line))
        {
            if (line.size() < 2 || line[0] != 'v' || line[1] != ' ')
            {
                continue;
            }

            std::istringstream iss(line.substr(2));
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            if (!(iss >> x >> y >> z))
            {
                continue;
            }

            has_vertex = true;
            min_x = std::min(min_x, x);
            min_y = std::min(min_y, y);
            min_z = std::min(min_z, z);
            max_x = std::max(max_x, x);
            max_y = std::max(max_y, y);
            max_z = std::max(max_z, z);
        }

        if (!has_vertex)
        {
            std::cerr << "Warning: No vertices found in mesh '" << mesh_filename
                      << "', using default bounding box (0.5m cube)" << std::endl;
            Eigen::Vector3d fallback(0.5, 0.5, 0.5);
            mesh_dims_cache.emplace(mesh_filename, fallback);
            return fallback;
        }

        Eigen::Vector3d dims(max_x - min_x, max_y - min_y, max_z - min_z);
        mesh_dims_cache.emplace(mesh_filename, dims);
        return dims;
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

    MapBounds XMLParser::computeMapBounds(const std::vector<StaticObject> &objects, double padding_factor)
    {
        if (objects.empty())
        {
            // Return default bounds if no objects
            MapBounds bounds;
            bounds.center = Eigen::Vector3d::Zero();
            bounds.size = Eigen::Vector3d(10.0, 10.0, 10.0);
            return bounds;
        }

        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double min_z = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        double max_z = -std::numeric_limits<double>::infinity();

        for (const auto &obj : objects)
        {
            // Skip planes as they don't contribute to meaningful bounds
            if (obj.type == ObjectType::PLANE)
            {
                continue;
            }

            // Get object extents (half-sizes)
            Eigen::Vector3d half_size = obj.dimensions * 0.5;
            if (obj.type == ObjectType::CYLINDER)
            {
                // For cylinder: dimensions = (radius, height, 0)
                half_size = Eigen::Vector3d(obj.dimensions.x(), obj.dimensions.x(), obj.dimensions.y() * 0.5);
            }

            // Compute object min/max bounds (simple AABB, ignoring rotation)
            Eigen::Vector3d obj_min = obj.position - half_size;
            Eigen::Vector3d obj_max = obj.position + half_size;

            min_x = std::min(min_x, obj_min.x());
            min_y = std::min(min_y, obj_min.y());
            min_z = std::min(min_z, obj_min.z());
            max_x = std::max(max_x, obj_max.x());
            max_y = std::max(max_y, obj_max.y());
            max_z = std::max(max_z, obj_max.z());
        }

        // Compute center and size
        Eigen::Vector3d center(
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5);

        Eigen::Vector3d size(
            (max_x - min_x) * padding_factor,
            (max_y - min_y) * padding_factor,
            (max_z - min_z) * padding_factor);

        MapBounds bounds;
        bounds.center = center;
        bounds.size = size;
        return bounds;
    }

} // namespace detection_mocker

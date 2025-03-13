#include "PlantArchitecture.h"
#include "CanopyGenerator.h"
#include "RadiationModel.h"
#include "Visualizer.h"
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <unordered_set>

using namespace helios;

/** Global parameters */
#define SAVE_FRUIT_3D_BBOX 1
#define SAVE_FRUIT_2D_BBOX 1
#define SAVE_RGB_IMAGES 1
#define SAVE_DEPTH_IMAGES 1
#define SAVE_LABEL_MAP 1
#define SAVE_CAMERA_CALIB 1
#define SAVE_CAMERA_POSES 1

#define ADD_OBSTACLES 0
#define ADD_FRUIT_VARIANCE 1
#define REMOVE_INTERSECTION 1
#define FIXED_CAMERA_POSE 0
#define SAMPLE_SEMISPHERE 1

// #define SOCKET_SERVER

/** Camera Parameters */
#define CAMERA_NAME "helios"
#define CAMERA_WIDTH 1024
#define CAMERA_HEIGHT 1024
#define CAMERA_FOV 55.0f
#define CAMERA_FOV_ASPECT_RATIO 1.0f
#define CAMERA_INIT_POS make_vec3(3, 0, 1.5)
#define CAMERA_INIT_LOOKAT make_vec3(0, 0, 1.5)
#define CAMERA_ANTIALIASING_SAMPLES 100
#define CAMERA_DEPTH_MAX 15.0f

/** Plant Parameters */
#define PLANT_SPACING_X 4.0f
#define PLANT_SPACING_Y 1.7f
#define PLANT_COUNT_X 1
#define PLANT_COUNT_Y 1
// #define PLANT_AGE 950.0f
#define PLANT_AGE 720.0f

/** Environment Parameters */
#define SUN_DIR_AZIMUTH deg2rad(45.0)
#define SUN_DIR_ELEVATION deg2rad(30.0)

/** Output Parameters */
std::string OUTPUT_DIR = "/media/peng/Samsung/Helios_results/apple_trees_one_tree_2_0/";
// std::string OUTPUT_DIR = "./";

enum class ObjectClass
{
    OTHER = 0,
    GROUND = 1,
    TREE = 2,
    LEAF = 3,
    FRUIT = 4, // overided by fruit_id
    WIRE = 5,
    POLE = 6,

    COUNT
};
constexpr int OBJECT_CLASS_COUNT = static_cast<int>(ObjectClass::COUNT);

// Socket server
#ifdef SOCKET_SERVER
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

int port = 8080;
int server_fd = -1;

std::string createTemporaryDirectory()
{
    char directory_name[] = "/tmp/helios_temp_dir.XXXXXX";
    char *directory = mkdtemp(directory_name);
    if (!directory)
    {
        perror("Failed to create temporary directory");
        return "";
    }
    std::cout << "Created temporary directory: " << directory << std::endl;
    return std::string(directory);
}

void cleanupTemporaryDirectory(const std::string &directory)
{
    std::string command = "rm -rf " + directory;
    system(command.c_str());
    std::cout << "Temporary directory cleaned up: " << directory << std::endl;
}

void setNonBlocking(int socket_fd)
{
    int flags = fcntl(socket_fd, F_GETFL, 0);
    if (flags == -1 || fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        perror("Failed to set non-blocking mode");
    }
}

int sendMessage(int client_socket, const std::string &message)
{
    if (send(client_socket, message.c_str(), message.size(), 0) < 0)
    {
        perror(("Failed to send message: " + message).c_str());
        return -1;
    }
    return 0;
}

int handleCameraCommands(int client_socket, Visualizer &visualizer, RadiationModel *radiation_model)
{

    std::string temp_dir = createTemporaryDirectory();
    temp_dir.append("/");

    constexpr size_t buffer_size = 4096;
    char buffer[buffer_size] = {0};

    std::string message;
    message.reserve(buffer_size);

    int flag = 0;

    // Set a timeout for select
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    int uuid = 0;

    while (true)
    {
        // Set up the file descriptor set
        fd_set readfds;
        FD_ZERO(&readfds);               // Clear the set
        FD_SET(client_socket, &readfds); // Add the client socket to the set

        // Wait for data on the socket
        int activity = select(client_socket + 1, &readfds, NULL, NULL, &timeout);
        if (activity < 0)
        {
            perror("Select error");
            flag = -1;
            break;
        }

        if (activity == 0 || !FD_ISSET(client_socket, &readfds))
        {
            continue; // No data, continue waiting
        }

        // Read data from the client
        int bytes_read = read(client_socket, buffer, buffer_size);
        if (bytes_read < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            perror("Read error");
            flag = -1;
            break;
        }
        else if (bytes_read == 0)
        {
            std::cout << "Connection closed by client." << std::endl;
            flag = 0;
            break;
        }

        message.assign(buffer, bytes_read);
        // std::cout << "Received command: " << command << std::endl;

        if (message == "EXIT")
        {
            std::cout << "Exit command received. Stopping server." << std::endl;
            flag = 1;
            memset(buffer, 0, bytes_read);
            break;
        }

        if (message == "OUTPUT_DIR")
        {
            sendMessage(client_socket, OUTPUT_DIR);
            memset(buffer, 0, bytes_read);
            continue;
        }

        if (message == "TEMP_DIR")
        {
            sendMessage(client_socket, temp_dir);
            memset(buffer, 0, bytes_read);
            continue;
        }

        if (message.compare(0, 10, "SET_CAMERA") == 0)
        {
            char *ptr = buffer + 10; // Skip "SET_CAMERA"
            char *end;

            float coords[6]; // {x, y, z, lx, ly, lz}
            bool valid = true;

            for (int i = 0; i < 6; i++)
            {
                coords[i] = strtof(ptr, &end);
                if (ptr == end)
                {
                    valid = false;
                    break;
                }
                ptr = end;
            }

            if (valid)
            {
                std::cout << "Updating camera to position (" << coords[0] << ", " << coords[1] << ", " << coords[2] << ") "
                          << "and look-at (" << coords[3] << ", " << coords[4] << ", " << coords[5] << ")." << std::endl;

                vec3 new_pos = make_vec3(coords[0], coords[1], coords[2]);
                vec3 new_lookat = make_vec3(coords[3], coords[4], coords[5]);

                if (SAVE_DEPTH_IMAGES || SAVE_LABEL_MAP || SAVE_FRUIT_2D_BBOX)
                {
                    // Update the radiation camera
                    radiation_model->setCameraPosition(CAMERA_NAME, new_pos);
                    radiation_model->setCameraLookat(CAMERA_NAME, new_lookat);
                    radiation_model->runCamera("simple");

                    if (SAVE_DEPTH_IMAGES)
                    {
                        radiation_model->writeNormDepthImage(CAMERA_NAME, "depth", CAMERA_DEPTH_MAX, temp_dir);
                        radiation_model->writeDepthImageData(CAMERA_NAME, "depthmap", temp_dir);
                    }

                    if (SAVE_LABEL_MAP)
                    {
                        radiation_model->writePrimitiveDataLabelMap(CAMERA_NAME, "class", "labelmap", temp_dir, -1, 0.0f);
                    }

                    if (SAVE_FRUIT_2D_BBOX)
                    {
                        radiation_model->writeImageBoundingBoxes_ObjectData(CAMERA_NAME, "fruit_id", 4, "bbox", temp_dir);
                    }
                }

                if (SAVE_CAMERA_POSES)
                {
                    std::ofstream cam_pose_file;
                    cam_pose_file.open(temp_dir + "camera_poses.txt");
                    cam_pose_file << int(0) << " "
                                  << coords[0] << " "
                                  << coords[1] << " "
                                  << coords[2] << " "
                                  << coords[3] << " "
                                  << coords[4] << " "
                                  << coords[5] << "\n";
                }

                // Update the visualizer
                visualizer.setCameraPosition(new_pos, new_lookat);
                visualizer.plotFastUpdate();

                if (SAVE_RGB_IMAGES)
                {
                    visualizer.printWindow((temp_dir + std::string(CAMERA_NAME) + "_rgb.jpg").c_str());
                }

                sendMessage(client_socket, std::to_string(uuid));
                uuid += 1;
            }
            else
            {
                std::cout << "Invalid command format." << std::endl;
                sendMessage(client_socket, "Invalid command format.");
            }
        }

        // Clear the buffer for the next message
        memset(buffer, 0, bytes_read);
    }

    cleanupTemporaryDirectory(temp_dir);
    close(client_socket);

    return flag;
}

int startSocketServer(Visualizer &visualizer, RadiationModel *radiation_model = nullptr)
{
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // Create socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("Socket failed");
        return EXIT_FAILURE;
    }

    // Set socket options
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0)
    {
        perror("setsockopt failed");
        close(server_fd);
        return EXIT_FAILURE;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    // Bind the socket
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        perror("Bind failed");
        close(server_fd);
        return EXIT_FAILURE;
    }

    // Listen for incoming connections
    if (listen(server_fd, 3) < 0)
    {
        perror("Listen failed");
        close(server_fd);
        return EXIT_FAILURE;
    }

    std::cout << "Server is listening on port " << port << std::endl;

    while (true)
    {
        int client_socket;

        // Accept an incoming connection
        if ((client_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
        {
            perror("Accept failed");
            continue;
        }

        std::cout << "Client connected. Waiting for commands..." << std::endl;

        // Set the socket to non-blocking mode
        setNonBlocking(client_socket);

        // Handle camera commands
        if (handleCameraCommands(client_socket, visualizer, radiation_model) > 0)
            break;
    }

    close(server_fd);
    return EXIT_SUCCESS;
}
#endif

int main()
{
    auto start_time = std::chrono::high_resolution_clock::now();

    /** Context */
    Context context;
    context.seedRandomGenerator(10);

    /** Generate ground */
    vec3 ground_origin(0, 0, 0);
    vec2 ground_extent(30, 50);
    int2 texture_subtiles(3, 5);
    int2 texture_subpatches(300, 300);
    float ground_rotation = 0;

    vec2 dx_tile(ground_extent.x / float(texture_subtiles.x), ground_extent.y / float(texture_subtiles.y));
    vec2 dx_subpatch(dx_tile.x / float(texture_subpatches.x), dx_tile.y / float(texture_subpatches.y));
    std::vector<uint> UUIDs;
    std::vector<uint> UUIDs_ground;
    for (int j = 0; j < texture_subtiles.y; j++)
    {
        for (int i = 0; i < texture_subtiles.x; i++)
        {
            vec3 center = ground_origin + make_vec3(-0.5f * ground_extent.x + (float(i) + 0.5f) * dx_tile.x, -0.5f * ground_extent.y + (float(j) + 0.5f) * dx_tile.y, 0);
            if (ground_rotation != 0)
            {
                center = rotatePointAboutLine(center, ground_origin, make_vec3(0, 0, 1), ground_rotation);
            }
            UUIDs = context.addTile(center, dx_tile, make_SphericalCoord(0, -ground_rotation), texture_subpatches, "plugins/visualizer/textures/dirt.jpg");
            UUIDs_ground.insert(UUIDs_ground.begin(), UUIDs.begin(), UUIDs.end());
        }
    }
    // auto UUIDs_ground = context.addPatch(nullorigin, make_vec2(30, 50));
    context.setPrimitiveData(UUIDs_ground, "twosided_flag", uint(0));

    /** Generate obstacles */
#if ADD_OBSTACLES && PLANT_COUNT_X * PLANT_COUNT_Y > 1
    float max_y = PLANT_SPACING_Y * (PLANT_COUNT_Y - 1) / 2.0;
    float min_y = -max_y;

    // Piles
    float mid_pole_width = 0.05;
    float mid_pole_height = 1.6;
    float end_pole_width = 0.07;
    float end_pole_height = 2.5;
    float end_pole_offset = 0.75;
    std::vector<uint> UUIDs_obs;
    std::vector<uint> UUIDs_poles;

    // Wires
    std::vector<float> wire_heights = {0.5, 1.0, 1.5};
    float wire_radius = 0.004;
    RGBcolor wire_color(0.3, 0.3, 0.3);
    vec3 wire_start_node(0, max_y + end_pole_offset, 0);
    vec3 wire_end_node(0, min_y - end_pole_offset, 0);
    float end_wire_span = 1.8;
    float end_wire_height = end_pole_height - 0.15;
    std::vector<uint> UUIDs_wires;

    for (float x = -PLANT_SPACING_X / 2.0; x >= -((PLANT_COUNT_X - 1) + 0.5) * PLANT_SPACING_X; x -= PLANT_SPACING_X)
    {
        // Piles
        for (float y = max_y - PLANT_SPACING_Y / 2.0; y >= min_y + PLANT_SPACING_Y / 2.0; y -= PLANT_SPACING_Y)
        {
            UUIDs_obs = context.addBox(
                vec3(x, y, mid_pole_height / 2),
                vec3(mid_pole_width, mid_pole_width, mid_pole_height),
                int3(1, 1, 1),
                "plugins/visualizer/textures/wood2.jpg");
            UUIDs_poles.insert(UUIDs_poles.begin(), UUIDs_obs.begin(), UUIDs_obs.end());
        }

        UUIDs_obs = context.addBox(
            vec3(x, max_y + end_pole_offset, end_pole_height / 2),
            vec3(end_pole_width, end_pole_width, end_pole_height),
            int3(1, 1, 1),
            "plugins/visualizer/textures/wood2.jpg");
        UUIDs_poles.insert(UUIDs_poles.begin(), UUIDs_obs.begin(), UUIDs_obs.end());

        UUIDs_obs = context.addBox(
            vec3(x, min_y - end_pole_offset, end_pole_height / 2),
            vec3(end_pole_width, end_pole_width, end_pole_height),
            int3(1, 1, 1),
            "plugins/visualizer/textures/wood2.jpg");
        UUIDs_poles.insert(UUIDs_poles.begin(), UUIDs_obs.begin(), UUIDs_obs.end());

        // Wires
        wire_start_node.x = x;
        wire_end_node.x = x;
        for (float wire_height : wire_heights)
        {
            wire_start_node.z = wire_height;
            wire_end_node.z = wire_height;
            UUIDs_obs = context.addTube(10, std::vector<vec3>{wire_start_node, wire_end_node}, std::vector<float>{wire_radius, wire_radius}, std::vector<RGBcolor>{wire_color, wire_color});
            UUIDs_wires.insert(UUIDs_wires.begin(), UUIDs_obs.begin(), UUIDs_obs.end());
        }

        UUIDs_obs = context.addTube(
            10,
            std::vector<vec3>{vec3(x, max_y + end_wire_span, 0), vec3(x, max_y + end_pole_offset, end_wire_height)},
            std::vector<float>{wire_radius, wire_radius},
            std::vector<RGBcolor>{wire_color, wire_color});
        UUIDs_wires.insert(UUIDs_wires.begin(), UUIDs_obs.begin(), UUIDs_obs.end());

        UUIDs_obs = context.addTube(
            10,
            std::vector<vec3>{vec3(x, min_y - end_wire_span, 0), vec3(x, min_y - end_pole_offset, end_wire_height)},
            std::vector<float>{wire_radius, wire_radius},
            std::vector<RGBcolor>{wire_color, wire_color});
        UUIDs_wires.insert(UUIDs_wires.begin(), UUIDs_obs.begin(), UUIDs_obs.end());
    }
#endif

    /** Plant architecture */
    std::cout << "Generating plants..." << std::endl;
    PlantArchitecture plantarchitecture(&context);
    plantarchitecture.loadPlantModelFromLibrary("apple");

    // Increase the leaf density
    ShootParameters shoot_parameters_proleptic = plantarchitecture.getCurrentShootParameters("proleptic");
    shoot_parameters_proleptic.internode_length_max = 0.035;
    // shoot_parameters_proleptic.max_nodes = 40;
    shoot_parameters_proleptic.max_nodes_per_season = 30;
    plantarchitecture.updateCurrentShootParameters("proleptic", shoot_parameters_proleptic);

    vec3 plant_origin(0, 0, 0);
    if (PLANT_COUNT_X * PLANT_COUNT_Y > 1)
        // plant_origin.x -= abs(PLANT_COUNT_X / 2.0f - 1.0f) * PLANT_SPACING_X; // move the origin, leave one row out for shadow
        plant_origin.x -= PLANT_COUNT_X / 2.0f * PLANT_SPACING_X; // move the origin

    std::vector<uint> UUIDs_plants = plantarchitecture.buildPlantCanopyFromLibrary(
        plant_origin, make_vec2(PLANT_SPACING_X, PLANT_SPACING_Y), make_int2(PLANT_COUNT_X, PLANT_COUNT_Y), PLANT_AGE);

    std::cout << "Number of plants: " << UUIDs_plants.size() << std::endl;
    std::cout << "Number of leaves: " << plantarchitecture.getAllLeafUUIDs().size() << std::endl;
    std::cout << "Generated plants successfully!" << std::endl;

#if REMOVE_INTERSECTION
    // Delete intersecting fruits
    for (uint plant_id : UUIDs_plants)
    {
        plantarchitecture.deleteIntersectingFruits(plant_id);
    }

#endif

    /** Total fruit count */
    int total_fruit_count = 0;
    for (int i = 0; i < UUIDs_plants.size(); i++)
    {
        std::vector<uint> fruit_Obj_UUIDs = plantarchitecture.getPlantFruitObjectIDs(UUIDs_plants[i]);
        for (int j = 0; j < fruit_Obj_UUIDs.size(); j++)
        {
            if (ADD_FRUIT_VARIANCE)
            {
                // Randomly scale the fruit
                context.scaleObjectAboutCenter(fruit_Obj_UUIDs[j], make_vec3(context.randn(1.0f, 0.05f), context.randn(1.0f, 0.05f), context.randn(1.0f, 0.05f)));

                // Randomly select texture
                std::vector<uint> fruit_primitive_UUIDs = context.getObjectPrimitiveUUIDs(fruit_Obj_UUIDs[j]);
                std::string texture_filename = "plugins/plantarchitecture/assets/textures/AppleFruit" + std::to_string(context.randu(1, 10)) + ".jpg";
                for (uint UUID : fruit_primitive_UUIDs)
                {
                    context.setPrimitiveTextureFile(UUID, texture_filename.c_str());
                }
            }

            context.setObjectData(fruit_Obj_UUIDs[j], "fruit_id", total_fruit_count);
            total_fruit_count += 1;
        }
    }
    std::cout << "Total fruit count: " << total_fruit_count << std::endl;

    /** Sun direction */
    SphericalCoord sun_dir_spherical = make_SphericalCoord(SUN_DIR_ELEVATION, SUN_DIR_AZIMUTH);
    vec3 sun_dir = sphere2cart(sun_dir_spherical);

#if SAVE_RGB_IMAGES
    /** Create directory for RGB images */
    std::string rgb_image_dir = OUTPUT_DIR + "rgb/";
    if (system(("mkdir -p " + rgb_image_dir).c_str()) != 0)
    {
        std::cerr << "Error creating directory: " << rgb_image_dir << std::endl;
    }
#endif

#if SAVE_DEPTH_IMAGES || SAVE_LABEL_MAP || SAVE_FRUIT_2D_BBOX || SAVE_FRUIT_3D_BBOX
    /** Create ground truth directory */
    std::string ground_truth_dir = OUTPUT_DIR + "ground_truth/";
    if (system(("mkdir -p " + ground_truth_dir).c_str()) != 0)
    {
        std::cerr << "Error creating directory: " << ground_truth_dir << std::endl;
    }
#endif

#if SAVE_FRUIT_3D_BBOX
    /** Save fruit bounding boxes */
    std::ofstream fruit_bbox_file;
    std::string fruit_bbox_filename = OUTPUT_DIR + "ground_truth/fruit_3d_bbox.txt";
    fruit_bbox_file.open(fruit_bbox_filename);
    fruit_bbox_file << "plant_id fruit_id center_x center_y center_z size_x size_y size_z\n";
    int fruit_id = 0;
    for (int i = 0; i < UUIDs_plants.size(); i++)
    {
        std::vector<uint> fruit_obj_UUIDs = plantarchitecture.getPlantFruitObjectIDs(UUIDs_plants[i]);
        for (int j = 0; j < fruit_obj_UUIDs.size(); j++)
        {
            vec3 min_corner, max_corner;
            context.getObjectBoundingBox(fruit_obj_UUIDs[j], min_corner, max_corner);
            vec3 center = (min_corner + max_corner) / 2.0f;
            vec3 size = (max_corner - min_corner) * 2.0f;
            context.getObjectData(fruit_obj_UUIDs[j], "fruit_id", fruit_id);
            fruit_bbox_file << i << " " << fruit_id << " "
                            << center.x << " " << center.y << " " << center.z << " "
                            << size.x << " " << size.y << " " << size.z << "\n";
        }
    }
    fruit_bbox_file.close();
#endif

#if SAVE_DEPTH_IMAGES || SAVE_LABEL_MAP || SAVE_FRUIT_2D_BBOX
    /** Radiation model */
    RadiationModel radiation_model(&context);
    radiation_model.disableMessages();

    // Radiation camera properties
    CameraProperties camera_properties;
    camera_properties.HFOV = CAMERA_FOV;
    camera_properties.camera_resolution = make_int2(CAMERA_WIDTH, CAMERA_HEIGHT);
    camera_properties.FOV_aspect_ratio = CAMERA_FOV_ASPECT_RATIO;
    camera_properties.lens_diameter = 0;
    camera_properties.focal_plane_distance = 1.0;

    // Set label data
    if (SAVE_LABEL_MAP)
    {
        context.setPrimitiveData(UUIDs_ground, "class", static_cast<int>(ObjectClass::GROUND));
        context.setPrimitiveData(plantarchitecture.getAllUUIDs(), "class", static_cast<int>(ObjectClass::TREE));
        context.setPrimitiveData(plantarchitecture.getAllLeafUUIDs(), "class", static_cast<int>(ObjectClass::LEAF));
        context.setPrimitiveData(plantarchitecture.getAllFruitUUIDs(), "class", static_cast<int>(ObjectClass::FRUIT));

#if ADD_OBSTACLES && PLANT_COUNT_X * PLANT_COUNT_Y > 1
        context.setPrimitiveData(UUIDs_wires, "class", static_cast<int>(ObjectClass::WIRE));
        context.setPrimitiveData(UUIDs_poles, "class", static_cast<int>(ObjectClass::POLE));
#endif

        int fruit_id;
        for (uint plant_id : UUIDs_plants)
        {
            for (uint fruit_obj_UUID : plantarchitecture.getPlantFruitObjectIDs(plant_id))
            {
                context.getObjectData(fruit_obj_UUID, "fruit_id", fruit_id);
                // The class value is offset by OBJECT_CLASS_COUNT from the fruit_id
                context.setPrimitiveData(context.getObjectPrimitiveUUIDs(fruit_obj_UUID), "class", fruit_id + OBJECT_CLASS_COUNT);
            }
        }
    }

    // Add a simple band
    radiation_model.addRadiationBand("simple");
    radiation_model.disableEmission("simple");
    radiation_model.setScatteringDepth("simple", 0);

    // Add a Radiation Camera
    radiation_model.addRadiationCamera(CAMERA_NAME, {"simple"}, CAMERA_INIT_POS, CAMERA_INIT_LOOKAT, camera_properties, CAMERA_ANTIALIASING_SAMPLES);
    radiation_model.runBand("simple");
#endif

#if SAVE_CAMERA_CALIB
    /** Save camera calibration to yaml */
    std::ofstream yaml_file;
    std::string yaml_filename = OUTPUT_DIR + "camera_calibration.yaml";
    yaml_file.open(yaml_filename);

    yaml_file << "camera_name: " << "\"" << CAMERA_NAME << "\"" << "\n"
              << "camera_model: " << "\"" << "PINHOLE" << "\"" << "\n"
              << "height: " << CAMERA_HEIGHT << "\n"
              << "width: " << CAMERA_WIDTH << "\n";

    double fx = CAMERA_WIDTH / 2.0 / std::tan(CAMERA_FOV * M_PI / 180.0 / 2.0);
    double fy = CAMERA_HEIGHT / 2.0 / std::tan(CAMERA_FOV / CAMERA_FOV_ASPECT_RATIO * M_PI / 180.0 / 2.0);
    yaml_file << "fx: " << std::fixed << std::setprecision(10) << fx << std::defaultfloat << "\n"
              << "fy: " << std::fixed << std::setprecision(10) << fy << std::defaultfloat << "\n"
              << "cx: " << std::fixed << std::setprecision(1) << float(CAMERA_WIDTH / 2.0) << std::defaultfloat << "\n"
              << "cy: " << std::fixed << std::setprecision(1) << float(CAMERA_HEIGHT / 2.0) << std::defaultfloat << "\n";

    yaml_file.close();
#endif

#if SAVE_CAMERA_POSES
    /** Save camera pose */
    std::ofstream cam_pose_file;
    std::string cam_pose_filename = OUTPUT_DIR + "ground_truth/camera_poses.txt";
    cam_pose_file.open(cam_pose_filename);
    cam_pose_file << "id pos_x pos_y pos_z lookat_x lookat_y lookat_z\n";
#endif

    /** Visualizer */
    Visualizer visualizer(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_ANTIALIASING_SAMPLES, false);
    visualizer.disableMessages();
    visualizer.hideWatermark();
    // RGBcolor sky_blue(0.53, 0.81, 0.92);
    RGBcolor sky_blue(0.69, 0.77, 0.87);
    visualizer.setBackgroundColor(sky_blue);
    // visualizer.addSkyDomeByCenter(100, make_vec3(0, 0, 0), 50, "plugins/visualizer/textures/SkyDome_clouds.jpg");
    visualizer.disableColorbar();
    visualizer.buildContextGeometry(&context);

    visualizer.setLightDirection(sun_dir);
    visualizer.setLightingModel(Visualizer::LIGHTING_PHONG_SHADOWED);
    visualizer.setLightIntensityFactor(1.0);

    visualizer.setCameraFieldOfView(CAMERA_FOV);
    visualizer.setCameraPosition(CAMERA_INIT_POS, CAMERA_INIT_LOOKAT);
    visualizer.plotFastUpdate();

#if !FIXED_CAMERA_POSE
    /** Generate multiple camera positions */
#if SAMPLE_SEMISPHERE && (PLANT_COUNT_X * PLANT_COUNT_Y == 1)
    auto random_generator = context.getRandomGenerator();
    float radius = 2.0f;
    std::uniform_real_distribution<float> cam_theta(0 * M_PI, 2.0f * M_PI);
    std::uniform_real_distribution<float> cam_z(0.5f, 4.0f);
    auto generate_random_position = [&]() -> vec3
    {
        float theta = cam_theta(*random_generator);
        // float z = cam_z(*random_generator);
        float z = 1.2;

        float r_xy = std::sqrt(radius * radius - z * z);
        float x = r_xy * std::cos(theta);
        float y = r_xy * std::sin(theta);

        return vec3(x, y, z);
    };

    std::vector<uint> all_plant_UUIDs = plantarchitecture.getAllUUIDs();
    std::map<uint, uint> fruit_id_to_obj_UUID;
    for (uint plant_UUID : UUIDs_plants)
    {
        std::vector<uint> fruit_obj_UUIDs = plantarchitecture.getPlantFruitObjectIDs(plant_UUID);
        for (uint fruit_obj_UUID : fruit_obj_UUIDs)
        {
            int fruit_id;
            context.getObjectData(fruit_obj_UUID, "fruit_id", fruit_id);
            fruit_id_to_obj_UUID[fruit_id] = fruit_obj_UUID;
        }
    }

    int total_samples = 30;
    int sample_count = 0;
    for (int i = 0; i < total_samples; i++)
    {
        // Sample random camera position
        vec3 sample_camera_position(generate_random_position());
        vec3 sample_lookat_position(0, 0, 1.0);

#else
    vec3 sample_camera_position(0, 0, 0);
    vec3 sample_lookat_position(0, 0, 0);
    std::vector<float> sample_heights = {1.0, 1.5, 2.0};

    float y_max = PLANT_SPACING_Y * (PLANT_COUNT_Y - 1) / 2.0;
    float y_min = -y_max;
    float y_step = PLANT_SPACING_Y;

    int sample_count = 0;
    int total_samples = PLANT_COUNT_Y * 3 * sample_heights.size();

    for (float current_y = y_max; current_y >= y_min; current_y -= y_step)
    {
        sample_lookat_position.x = -PLANT_SPACING_X / 2.0;
        sample_lookat_position.y = current_y;

        for (float y_offset = PLANT_SPACING_Y / 2.0; y_offset >= -PLANT_SPACING_Y / 2.0; y_offset -= PLANT_SPACING_Y / 2.0f)
        {
            sample_camera_position.y = current_y + y_offset;

            for (auto current_z : sample_heights)
            {
                sample_camera_position.z = current_z;
                sample_lookat_position.z = current_z - 0.2 * (current_z - 1.5); // add variance to height
#endif

#if SAVE_DEPTH_IMAGES || SAVE_LABEL_MAP || SAVE_FRUIT_2D_BBOX
        radiation_model.setCameraPosition(CAMERA_NAME, sample_camera_position);
        radiation_model.setCameraLookat(CAMERA_NAME, sample_lookat_position);
        radiation_model.updateGeometry();
        radiation_model.runCamera("simple");

        if (SAVE_DEPTH_IMAGES)
        {
            radiation_model.writeNormDepthImage(CAMERA_NAME, "frame", CAMERA_DEPTH_MAX, OUTPUT_DIR + "depth/", sample_count);
            radiation_model.writeDepthImageData(CAMERA_NAME, "frame", OUTPUT_DIR + "ground_truth/depth_map/", sample_count);
        }

        if (SAVE_LABEL_MAP)
        {
            radiation_model.writePrimitiveDataLabelMap(CAMERA_NAME, "class", "frame", OUTPUT_DIR + "ground_truth/label_map/", sample_count, 0.0f);
        }

        if (SAVE_FRUIT_2D_BBOX)
        {
            radiation_model.writeImageBoundingBoxes_ObjectData(CAMERA_NAME, "fruit_id", 4, "frame", OUTPUT_DIR + "ground_truth/fruit_bbox/", false, sample_count);
        }
#endif
        // RGB rendering
        visualizer.setCameraPosition(sample_camera_position, sample_lookat_position);
        visualizer.plotFastUpdate();

#if SAVE_RGB_IMAGES
        std::ostringstream oss;
        oss << std::setw(5) << std::setfill('0') << sample_count;
        std::string filename = rgb_image_dir + CAMERA_NAME + "_frame_" + oss.str() + ".jpg";
        visualizer.printWindow(filename.c_str());
#endif

#if SAVE_LABEL_MAP
        std::string label_map_filename = OUTPUT_DIR + "ground_truth/label_map/helios_frame_" +
                                         (std::stringstream() << std::setw(5) << std::setfill('0') << sample_count).str() + ".txt";
        std::ifstream file(label_map_filename);
        std::unordered_set<int> unique_values;
        int value;
        while (file >> value)
            unique_values.insert(value);
        file.close();

        std::vector<uint> visible_fruit_ids;
        for (int value : unique_values)
        {
            if (value >= OBJECT_CLASS_COUNT)
            {
                visible_fruit_ids.push_back(value - OBJECT_CLASS_COUNT);
            }
        }

        context.hidePrimitive(all_plant_UUIDs);
        uint last_show_obj_UUID;
        for (int i = 0; i < visible_fruit_ids.size(); i++)
        {
            uint fruit_obj_UUID = fruit_id_to_obj_UUID[visible_fruit_ids[i]];
            if (i != 0)
                context.hideObject(std::vector<uint>{last_show_obj_UUID});
            context.showObject(std::vector<uint>{fruit_obj_UUID});
            last_show_obj_UUID = fruit_obj_UUID;

            radiation_model.updateGeometry();
            radiation_model.runCamera("simple");

            radiation_model.writePrimitiveDataLabelMap(CAMERA_NAME, "class", "frame", OUTPUT_DIR + "ground_truth/fruit_id_" + std::to_string(visible_fruit_ids[i]) + "_label_map/", sample_count, 0.0f);
        }
        context.showPrimitive(all_plant_UUIDs);

#endif

#if SAVE_CAMERA_POSES
        cam_pose_file << sample_count << " "
                      << sample_camera_position.x << " "
                      << sample_camera_position.y << " "
                      << sample_camera_position.z << " "
                      << sample_lookat_position.x << " "
                      << sample_lookat_position.y << " "
                      << sample_lookat_position.z << "\n";
#endif

        sample_count++;

        if (sample_count % 10 == 0)
            printf("Processed %d/%d samples\n", sample_count, total_samples);

#if SAMPLE_SEMISPHERE && (PLANT_COUNT_X * PLANT_COUNT_Y == 1)
#else
            }
        }
#endif
    }

#else

#if SAVE_CAMERA_POSES
    cam_pose_file << int(0) << " "
                  << CAMERA_INIT_POS.x << " "
                  << CAMERA_INIT_POS.y << " "
                  << CAMERA_INIT_POS.z << " "
                  << CAMERA_INIT_LOOKAT.x << " "
                  << CAMERA_INIT_LOOKAT.y << " "
                  << CAMERA_INIT_LOOKAT.z << "\n";
#endif

#if SAVE_DEPTH_IMAGES || SAVE_LABEL_MAP || SAVE_FRUIT_2D_BBOX
    // radiation_model.setCameraPosition(CAMERA_NAME, CAMERA_INIT_POS);
    // radiation_model.setCameraLookat(CAMERA_NAME, CAMERA_INIT_LOOKAT);
    // radiation_model.runCamera("simple");

    // Depth image and map
    if (SAVE_DEPTH_IMAGES)
    {
        radiation_model.writeNormDepthImage(CAMERA_NAME, "depth", CAMERA_DEPTH_MAX);
        radiation_model.writeDepthImageData(CAMERA_NAME, "depthmap");
    }

    // Label map
    if (SAVE_LABEL_MAP)
    {
        radiation_model.writePrimitiveDataLabelMap(CAMERA_NAME, "class", "labelmap", "./", -1, 0.0f);
    }

    // Fruit 2D bounding box
    if (SAVE_FRUIT_2D_BBOX && total_fruit_count > 0)
    {
        radiation_model.writeImageBoundingBoxes_ObjectData(CAMERA_NAME, "fruit_id", 4, "bbox");
    }
#endif

    // RGB
    // visualizer.plotFastUpdate();

    if (SAVE_RGB_IMAGES)
    {
        visualizer.printWindow((std::string(CAMERA_NAME) + "_rgb.jpg").c_str());
    }

#ifdef SOCKET_SERVER
    // signal(SIGINT, signalHandler);

#if SAVE_DEPTH_IMAGES || SAVE_LABEL_MAP || SAVE_FRUIT_2D_BBOX
    int status = startSocketServer(visualizer, &radiation_model);
#else
    int status = startSocketServer(visualizer);
#endif
#endif

    // Remain interactive window
    visualizer.plotInteractive();

#endif

#if SAVE_CAMERA_POSES
    if (cam_pose_file.is_open())
        cam_pose_file.close();
#endif

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    std::cout << "Elapsed time: " << duration << " seconds" << std::endl;

    return 0;
}
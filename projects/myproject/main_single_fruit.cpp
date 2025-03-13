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
#define SAVE_FRUIT_2D_BBOX 1
#define SAVE_RGB_IMAGE 1
#define SAVE_DEPTH_IMAGE 1
#define SAVE_LABEL_MAP 1
#define SAVE_CAMERA_CALIB 1
#define SAVE_CAMERA_POSE 1
#define SAVE_TO_OBJ 1

#define ADD_FRUIT_VARIANCE 0
#define FIXED_CAMERA_POSE 1

/** Camera Parameters */
#define CAMERA_NAME "helios"
#define CAMERA_WIDTH 800
#define CAMERA_HEIGHT 800
#define CAMERA_FOV 55.0f
#define CAMERA_FOV_ASPECT_RATIO 1.0f
#define CAMERA_INIT_POS make_vec3(1.5, 0, 2)
#define CAMERA_INIT_LOOKAT make_vec3(0, 0, 2)
#define CAMERA_ANTIALIASING_SAMPLES 100
#define CAMERA_DEPTH_MAX 15.0f

/** Environment Parameters */
#define SUN_DIR_AZIMUTH deg2rad(45.0)
#define SUN_DIR_ELEVATION deg2rad(30.0)

/** Output Parameters */
std::string OUTPUT_DIR = "/media/peng/Samsung/Helios_results/one_apple/";
// std::string OUTPUT_DIR = "./";

int main()
{
    auto start_time = std::chrono::high_resolution_clock::now();

    /** Context */
    Context context;
    context.seedRandomGenerator(10);

    /** Generate one Apple */
    std::vector<uint> apple_UUIDs = context.loadOBJ("plugins/plantarchitecture/assets/obj/AppleFruit_New.obj", make_vec3(0.0, 0.0, 0.0), 0, nullrotation, RGB::red, "ZUP", true);
    uint apple_objID = context.addPolymeshObject(apple_UUIDs);

    vec3 fruit_position(0, 0, 2.0);
    vec3 fruit_scale(0.1f, 0.1f, 0.1f);
    context.scaleObject(apple_objID, fruit_scale);
    context.rotateObject(apple_objID, M_PI / 2, "y");
    context.translateObject(apple_objID, fruit_position);

    vec3 min_corner, max_corner;
    context.getObjectBoundingBox(apple_objID, min_corner, max_corner);
    vec3 center = (min_corner + max_corner) / 2.0f;
    vec3 size = (max_corner - min_corner) * 2.0f;

    std::cout << "Fruit position: " << fruit_position << std::endl;
    std::cout << "Fruit size: " << size << std::endl;

    if (ADD_FRUIT_VARIANCE)
    {
        // Randomly scale fruits
        context.scaleObjectAboutCenter(apple_objID, make_vec3(context.randn(1.0f, 0.05f), context.randn(1.0f, 0.05f), context.randn(1.0f, 0.05f)));

        // Randomly select texture
        std::vector<uint> apple_primitive_UUIDs = context.getObjectPrimitiveUUIDs(apple_objID);
        std::string texture_filename = "plugins/plantarchitecture/assets/textures/AppleFruit" + std::to_string(context.randu(1, 10)) + ".jpg";
        for (uint UUID : apple_primitive_UUIDs)
        {
            context.setPrimitiveTextureFile(UUID, texture_filename.c_str());
        }
    }

    // context.overrideObjectTextureColor(apple_objID);

    /** Sun direction */
    SphericalCoord sun_dir_spherical = make_SphericalCoord(SUN_DIR_ELEVATION, SUN_DIR_AZIMUTH);
    vec3 sun_dir = sphere2cart(sun_dir_spherical);

#if SAVE_RGB_IMAGE
    /** Create directory for RGB images */
    std::string rgb_image_dir = OUTPUT_DIR + "rgb/";
    if (system(("mkdir -p " + rgb_image_dir).c_str()) != 0)
    {
        std::cerr << "Error creating directory: " << rgb_image_dir << std::endl;
    }
#endif

#if SAVE_DEPTH_IMAGE || SAVE_LABEL_MAP || SAVE_FRUIT_2D_BBOX
    /** Create ground truth directory */
    std::string ground_truth_dir = OUTPUT_DIR + "ground_truth/";
    if (system(("mkdir -p " + ground_truth_dir).c_str()) != 0)
    {
        std::cerr << "Error creating directory: " << ground_truth_dir << std::endl;
    }
#endif

#if SAVE_DEPTH_IMAGE || SAVE_LABEL_MAP || SAVE_FRUIT_2D_BBOX
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
        context.setPrimitiveData(context.getObjectPrimitiveUUIDs(apple_objID), "class", 1);
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

#if SAVE_CAMERA_POSE
    /** Save camera pose */
    std::ofstream cam_pose_file;
    std::string cam_pose_filename = OUTPUT_DIR + "ground_truth/camera_poses.txt";
    cam_pose_file.open(cam_pose_filename);
    cam_pose_file << "id pos_x pos_y pos_z lookat_x lookat_y lookat_z\n";
#endif

#if SAVE_TO_OBJ
    // Save to .obj file
    std::string obj_dir = OUTPUT_DIR + "object/";
    if (system(("mkdir -p " + obj_dir).c_str()) != 0)
    {
        std::cerr << "Error creating directory: " << obj_dir << std::endl;
    }
    context.writeOBJ(obj_dir + "scene.obj");
#endif

    /** Visualizer */
    Visualizer visualizer(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_ANTIALIASING_SAMPLES, false);
    visualizer.disableMessages();
    visualizer.hideWatermark();
    RGBcolor sky_blue(0.0, 0.0, 0.0);
    visualizer.setBackgroundColor(sky_blue);
    visualizer.disableColorbar();
    visualizer.buildContextGeometry(&context);

    visualizer.setLightDirection(sun_dir);
    visualizer.setLightingModel(Visualizer::LIGHTING_NONE);
    visualizer.setLightIntensityFactor(1.0);

    visualizer.setCameraFieldOfView(CAMERA_FOV);
    visualizer.setCameraPosition(CAMERA_INIT_POS, CAMERA_INIT_LOOKAT);
    visualizer.plotFastUpdate();

#if !FIXED_CAMERA_POSE
    /** Sample camera positions */
    auto random_generator = context.getRandomGenerator();
    float radius = 0.3f;
    std::uniform_real_distribution<float> theta_dist(0, 2.0f * M_PI); // azimuthal angle
    std::uniform_real_distribution<float> phi_dist(0, M_PI);          // polar angle
    auto generate_random_position = [&]() -> vec3
    {
        float theta = theta_dist(*random_generator); // azimuthal angle [0, 2π]
        float phi = phi_dist(*random_generator);     // polar angle [0, π]

        // Spherical to Cartesian conversion
        float x = radius * std::sin(phi) * std::cos(theta) + fruit_position.x;
        float y = radius * std::sin(phi) * std::sin(theta) + fruit_position.y;
        float z = radius * std::cos(phi) + fruit_position.z;

        return vec3(x, y, z);
    };

    int total_samples = 100;
    int sample_count = 0;
    for (int i = 0; i < total_samples; i++)
    {
        // Sample random camera position
        vec3 sample_camera_position(generate_random_position());
        vec3 sample_lookat_position = fruit_position;

#if SAVE_DEPTH_IMAGE || SAVE_LABEL_MAP || SAVE_FRUIT_2D_BBOX
        radiation_model.setCameraPosition(CAMERA_NAME, sample_camera_position);
        radiation_model.setCameraLookat(CAMERA_NAME, sample_lookat_position);
        radiation_model.updateGeometry();
        radiation_model.runCamera("simple");

        if (SAVE_DEPTH_IMAGE)
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

#if SAVE_RGB_IMAGE
        std::ostringstream oss;
        oss << std::setw(5) << std::setfill('0') << sample_count;
        std::string filename = rgb_image_dir + CAMERA_NAME + "_frame_" + oss.str() + ".jpg";
        visualizer.printWindow(filename.c_str());
#endif

#if SAVE_CAMERA_POSE
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
    }
#else

#if SAVE_CAMERA_POSE
    cam_pose_file << int(0) << " "
                  << CAMERA_INIT_POS.x << " "
                  << CAMERA_INIT_POS.y << " "
                  << CAMERA_INIT_POS.z << " "
                  << CAMERA_INIT_LOOKAT.x << " "
                  << CAMERA_INIT_LOOKAT.y << " "
                  << CAMERA_INIT_LOOKAT.z << "\n";
#endif

#if SAVE_DEPTH_IMAGE || SAVE_LABEL_MAP || SAVE_FRUIT_2D_BBOX
    // radiation_model.setCameraPosition(CAMERA_NAME, CAMERA_INIT_POS);
    // radiation_model.setCameraLookat(CAMERA_NAME, CAMERA_INIT_LOOKAT);
    // radiation_model.runCamera("simple");

    // Depth image and map
    if (SAVE_DEPTH_IMAGE)
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
    if (SAVE_FRUIT_2D_BBOX)
    {
        radiation_model.writeImageBoundingBoxes_ObjectData(CAMERA_NAME, "fruit_id", 4, "bbox");
    }
#endif

#if SAVE_RGB_IMAGE
    visualizer.plotFastUpdate();
    visualizer.printWindow((std::string(CAMERA_NAME) + "_rgb.jpg").c_str());
#endif

    // Remain interactive window
    visualizer.plotInteractive();

#endif

#if SAVE_CAMERA_POSE
    if (cam_pose_file.is_open())
        cam_pose_file.close();
#endif

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    std::cout << "Elapsed time: " << duration << " seconds" << std::endl;

    return 0;
}
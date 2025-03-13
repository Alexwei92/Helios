#include "PlantArchitecture.h"
#include "CanopyGenerator.h"
#include "RadiationModel.h"
#include "Visualizer.h"
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>

using namespace helios;

/** Global parameters */
#define ADD_GROUND 0             // add ground
#define ADD_OBSTACLES 0          // add wood poles and wires
#define ADD_FRUIT_VARIANCE 0     // add variance to fruit size and
#define REMOVE_FRUIT_INTERSECT 0 // remove intersecting fruits
#define SAVE_TO_OBJ 1            // save scene to .obj

/** Camera Parameters */
#define CAMERA_NAME "helios"
#define CAMERA_WIDTH 800
#define CAMERA_HEIGHT 800
#define CAMERA_FOV 55.0f // degrees
#define CAMERA_FOV_ASPECT_RATIO 1.0f
#define CAMERA_INIT_POS make_vec3(3, 0, 1.5)    // meters
#define CAMERA_INIT_LOOKAT make_vec3(0, 0, 1.5) // meters
#define CAMERA_ANTIALIASING_SAMPLES 100

/** Plant Parameters */
#define PLANT_SPACING_X 4.5f // meters
#define PLANT_SPACING_Y 3.0f // meters
// #define PLANT_SPACING_Y 1.8f // meters
#define PLANT_COUNT_X 1
#define PLANT_COUNT_Y 1
// #define PLANT_AGE 730.0f // days (apple)
// #define PLANT_AGE 1272.0f // days (almond)
#define PLANT_AGE 60.0f // days (strawberry)
// #define PLANT_AGE 66.0f // days (tomato)

/** Environment Parameters */
#define SUN_DIR_AZIMUTH deg2rad(45.0)
#define SUN_DIR_ELEVATION deg2rad(30.0)

/** Output Parameters */
std::string OUTPUT_DIR = "/home/peng/Samsung/strawberry/plant10/";

int main()
{
    auto start_time = std::chrono::high_resolution_clock::now();

    /** Context */
    Context context;
    context.seedRandomGenerator(10);

    /** Generate ground */
#if ADD_GROUND
    vec3 ground_origin(0, 0, 0);
    vec2 ground_extent(70, 50);
    int2 texture_subtiles(7, 5);
    int2 texture_subpatches(300, 300);
    float ground_rotation = 0;

    vec2 dx_tile(ground_extent.x / float(texture_subtiles.x), ground_extent.y / float(texture_subtiles.y));
    vec2 dx_subpatch(dx_tile.x / float(texture_subpatches.x), dx_tile.y / float(texture_subpatches.y));
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
            std::vector<uint> UUIDs_tile = context.addTile(center, dx_tile, make_SphericalCoord(0, -ground_rotation), texture_subpatches, "plugins/visualizer/textures/dirt.jpg");
            UUIDs_ground.insert(UUIDs_ground.begin(), UUIDs_tile.begin(), UUIDs_tile.end());
        }
    }
    // auto UUIDs_ground = context.addPatch(nullorigin, ground_extent);
#endif

    /** Generate obstacles */
#if ADD_OBSTACLES && PLANT_COUNT_X * PLANT_COUNT_Y > 1
    float max_y = PLANT_SPACING_Y * (PLANT_COUNT_Y - 1) / 2.0;
    float min_y = -max_y;

    // Poles
    float mid_pole_width = 0.06; // 0.05
    float mid_pole_height = 2.5; // 1.6
    float end_pole_width = 0.07;
    float end_pole_height = 2.5;
    float end_pole_offset = PLANT_SPACING_Y / 2.0; // 0.75
    std::vector<uint> UUIDs_obs;
    std::vector<uint> UUIDs_poles;

    // Wires
    std::vector<float> wire_heights = {0.5, 1.0, 1.5, 2.0};
    float wire_radius = 0.004;
    RGBcolor wire_color(0.3, 0.3, 0.3);
    vec3 wire_start_node(0, max_y + end_pole_offset, 0);
    vec3 wire_end_node(0, min_y - end_pole_offset, 0);
    float end_wire_span = 2.0;
    float end_wire_height = end_pole_height - 0.15;
    std::vector<uint> UUIDs_wires;

    for (float x = -PLANT_SPACING_X / 2.0; x >= -((PLANT_COUNT_X - 1) + 0.5) * PLANT_SPACING_X; x -= PLANT_SPACING_X)
    {
        // Poles
        for (float y = max_y - PLANT_SPACING_Y / 2.0; y >= (min_y + PLANT_SPACING_Y / 2.0f - 1e-3); y -= PLANT_SPACING_Y)
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
    plantarchitecture.loadPlantModelFromLibrary("strawberry");

    // // Increase the leaf density
    // ShootParameters shoot_parameters_proleptic = plantarchitecture.getCurrentShootParameters("proleptic");
    // shoot_parameters_proleptic.internode_length_max = 0.035;
    // // shoot_parameters_proleptic.max_nodes = 40;
    // shoot_parameters_proleptic.max_nodes_per_season = 30;
    // plantarchitecture.updateCurrentShootParameters("proleptic", shoot_parameters_proleptic);

    vec3 plant_origin(0, 0, 0);
    if (PLANT_COUNT_X * PLANT_COUNT_Y > 1)
        // plant_origin.x -= abs(PLANT_COUNT_X / 2.0f - 1.0f) * PLANT_SPACING_X; // move the origin, leave one row out for shadow
        plant_origin.x -= PLANT_COUNT_X / 2.0f * PLANT_SPACING_X; // move the origin

    std::vector<uint> UUIDs_plants = plantarchitecture.buildPlantCanopyFromLibrary(
        plant_origin, make_vec2(PLANT_SPACING_X, PLANT_SPACING_Y), make_int2(PLANT_COUNT_X, PLANT_COUNT_Y), PLANT_AGE);

    std::cout << "Number of plants: " << UUIDs_plants.size() << std::endl;
    std::cout << "Number of leaves: " << plantarchitecture.getAllLeafUUIDs().size() << std::endl;
    std::cout << "Number of flowers: " << plantarchitecture.getAllFlowerUUIDs().size() << std::endl;
    std::cout << "Generated plants successfully!" << std::endl;

#if REMOVE_FRUIT_INTERSECT
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
                // Randomly scale fruits
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

#if SAVE_TO_OBJ
    // Save to .obj file
    std::string obj_dir = OUTPUT_DIR;
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
    // RGBcolor sky_blue(0.53, 0.81, 0.92);
    RGBcolor sky_blue(0.69, 0.77, 0.87);
    visualizer.setBackgroundColor(sky_blue);
    visualizer.addSkyDomeByCenter(200, make_vec3(0, 0, 0), 100, "plugins/visualizer/textures/SkyDome_clouds.jpg");
    visualizer.disableColorbar();
    visualizer.buildContextGeometry(&context);

    visualizer.setLightDirection(sun_dir);
    visualizer.setLightingModel(Visualizer::LIGHTING_PHONG_SHADOWED);
    visualizer.setLightIntensityFactor(1.0);

    visualizer.setCameraFieldOfView(CAMERA_FOV);
    visualizer.setCameraPosition(CAMERA_INIT_POS, CAMERA_INIT_LOOKAT);
    visualizer.plotFastUpdate();

    // Remain interactive window
    visualizer.plotInteractive();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    std::cout << "Elapsed time: " << duration << " seconds" << std::endl;

    return 0;
}
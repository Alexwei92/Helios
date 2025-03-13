#include "CanopyGenerator.h"
#include "RadiationModel.h"
#include "Visualizer.h"

// #define USE_SOLARPOSITION_PLUGIN
#ifdef USE_SOLARPOSITION_PLUGIN
#include "SolarPosition.h"
#endif

using namespace helios;

int main()
{

  Context context;

  // Generate ground
  CanopyGenerator canopy_generator(&context);
  canopy_generator.buildGround(
      nullorigin,
      make_vec2(100, 100),
      make_int2(5, 5),
      make_int2(300, 300),
      "plugins/visualizer/textures/dirt.jpg");
  std::vector<uint> UUIDs_ground = canopy_generator.getGroundUUIDs();

  // Generate plant
  WalnutCanopyParameters parameters;
  parameters.plant_count = make_int2(1, 1);
  canopy_generator.seedRandomGenerator(10);
  canopy_generator.buildCanopy(parameters);
  std::vector<uint> UUIDs_trunk = canopy_generator.getTrunkUUIDs();
  std::vector<uint> UUIDs_branch = canopy_generator.getBranchUUIDs();
  std::vector<uint> UUIDs_leaf = canopy_generator.getLeafUUIDs();
  std::vector<uint> UUIDs_fruit = canopy_generator.getFruitUUIDs();

  // Set Class
  context.setPrimitiveData(UUIDs_ground, "class", 0);
  context.setPrimitiveData(UUIDs_trunk, "class", 1);
  context.setPrimitiveData(UUIDs_branch, "class", 1);
  context.setPrimitiveData(UUIDs_leaf, "class", 2);
  context.setPrimitiveData(UUIDs_fruit, "class", 3);

//   // Plot fruit bounding boxes
//   for (uint plant_ID = 0; plant_ID < canopy_generator.getPlantCount(); plant_ID++)
//   {
//     std::vector<std::vector<uint>> fruit_UUIDs = canopy_generator.getFruitUUIDs(plant_ID).at(0);
//     std::cout << "Plant ID: " << plant_ID << " has " << fruit_UUIDs.size() << " fruits." << std::endl;
//     for (auto UUID : fruit_UUIDs)
//     {
//       vec3 min_corner, max_corner;
//       context.getPrimitiveBoundingBox(UUID, min_corner, max_corner);

//       vec3 center = (min_corner + max_corner) / 2.0f;
//       vec3 size = (max_corner - min_corner) * 2.0f;
//       context.addBox(center, size, make_int3(1, 1, 1), RGB::red);
//     }
//   }

  // Load and set spectral reflectivity data
  context.loadXML("plugins/radiation/spectral_data/surface_spectral_library.xml");
  context.setPrimitiveData(UUIDs_ground, "reflectivity_spectrum", "soil_reflectivity_0000");
  context.setPrimitiveData(UUIDs_trunk, "reflectivity_spectrum", "bark_reflectivity_0000");
  context.setPrimitiveData(UUIDs_branch, "reflectivity_spectrum", "bark_reflectivity_0000");
  // context.setPrimitiveData(UUIDs_leaf, "reflectivity_spectrum", "walnut_leaf_reflectivity_0000");
  context.setPrimitiveData(UUIDs_leaf, "reflectivity_spectrum", "tomato_leaf_reflectivity_0000");
  context.setPrimitiveData(UUIDs_leaf, "transmissivity_spectrum", "tomato_leaf_transmissivity_0000");
  context.setPrimitiveData(UUIDs_fruit, "reflectivity_spectrum", "walnut_hull_reflectivity_0000");

  // Radiation model
  RadiationModel radiation_model(&context);

// Add a sun radiation source
#ifdef USE_SOLARPOSITION_PLUGIN
  SolarPosition solar_position(7, 35.2, 124.3, &context);
  context.setDate(make_Date(10, 1, 2024));
  context.setTime(0, 11);

  uint sun_source_ID = radiation_model.addCollimatedRadiationSource();
  vec3 sun_dir = solar_position.getSunDirectionVector();
  radiation_model.setSourcePosition(sun_source_ID, sun_dir);
  radiation_model.setSourceSpectrum(sun_source_ID, "solar_spectrum_direct_ASTMG173");
#else
  vec3 sun_dir = sphere2cart(make_SphericalCoord(deg2rad(45), deg2rad(0)));
  uint sun_source_ID = radiation_model.addSunSphereRadiationSource(sun_dir);
  radiation_model.setSourceSpectrum(sun_source_ID, "solar_spectrum_direct_ASTMG173");
#endif

  // Add R,G,B bands
  radiation_model.addRadiationBand("red");
  radiation_model.disableEmission("red");
  radiation_model.setScatteringDepth("red", 5);

#ifdef USE_SOLARPOSITION_PLUGIN
  float pressure = 101300; // atmospheric pressure (Pa)
  float temperature = 300; // air temperature (K)
  float humidity = 0.5;    // relative humidity (0-1)
  float turbidity = 0.05;  // atmospheric turbidity coeff (-)
  float Rflux = solar_position.getSolarFlux(pressure, temperature, humidity, turbidity);
  float fdiff = solar_position.getDiffuseFraction(pressure, temperature, humidity, turbidity);

  radiation_model.setDirectRayCount("red", 1000);
  radiation_model.setSourceFlux(sun_source_ID, "red", Rflux * (1 - fdiff));
  radiation_model.setDiffuseRadiationFlux("red", Rflux * fdiff);
#else
  radiation_model.setSourceFlux(sun_source_ID, "red", 1000.0f);
  radiation_model.setDiffuseRadiationFlux("red", 400.0f);
#endif

  radiation_model.copyRadiationBand("red", "green");
  radiation_model.copyRadiationBand("red", "blue");

  // We only want the ground to absorb radiation from the top.
  context.setPrimitiveData(UUIDs_ground, "twosided_flag", uint(0));

  // // Use periodic lateral boundaries so we have repeating trees
  // radiation_model.enforcePeriodicBoundary("xy");

  radiation_model.updateGeometry();

  // Add a Radiation Camera
  CameraProperties camera_properties;
  camera_properties.HFOV = 69.4;
  camera_properties.FOV_aspect_ratio = 1.633;
  camera_properties.camera_resolution = make_int2(848, 480);
  camera_properties.lens_diameter = 0.94e-3;
  camera_properties.focal_plane_distance = 1.88;

  vec3 camera_position(6, 0, 6);
  vec3 camera_lookat(0, 0, 3);
  uint antialiasing_samples = 100;
  radiation_model.addRadiationCamera("visible", {"red", "green", "blue"}, camera_position, camera_lookat, camera_properties, antialiasing_samples);

  // Load and set calibrated camera spectral response for Nikon B500 camera and ASTMG173 solar spectrum
  context.loadXML("plugins/radiation/spectral_data/camera_spectral_library.xml");
  radiation_model.setCameraSpectralResponse("visible", "red", "calibrated_sun_NikonB500_spectral_response_red");
  radiation_model.setCameraSpectralResponse("visible", "green", "calibrated_sun_NikonB500_spectral_response_green");
  radiation_model.setCameraSpectralResponse("visible", "blue", "calibrated_sun_NikonB500_spectral_response_blue");

  radiation_model.runBand({"red", "green", "blue"});
  // radiation_model.writeCameraImage("visible", {"red", "green", "blue"}, "rgb");
  // radiation_model.writeNormDepthImage("visible", "depth", 12);

  // // Generate random camera positions on a semi-sphere
  // auto random_generator = context.getRandomGenerator();
  // float radius = 4.0f;
  // std::uniform_real_distribution<float> cam_theta(0.0f, 2.0f * M_PI);
  // std::uniform_real_distribution<float> cam_z(0.5f, 3.0f);
  // auto generate_random_position = [&]() -> vec3
  // {
  //   float theta = cam_theta(*random_generator);
  //   float z = cam_z(*random_generator);

  //   float r_xy = std::sqrt(radius * radius - z * z);
  //   float x = r_xy * std::cos(theta);
  //   float y = r_xy * std::sin(theta);

  //   return vec3(x, y, z);
  // };

  // std::uniform_real_distribution<float> lookat_z(1.0f, 4.5f);
  // uint num_samples = 50;
  // for (int i = 0; i < num_samples; i++)
  // {
  //   // Sample random camera position
  //   vec3 sample_camera_position(generate_random_position());
  //   vec3 sample_lookat_position(0, 0, lookat_z(*random_generator));

  //   radiation_model.setCameraPosition("visible", sample_camera_position);
  //   radiation_model.runBand({"red", "green", "blue"});

  //   // Save RGB and depth images
  //   radiation_model.writeCameraImage("visible", {"red", "green", "blue"}, "rgb", "/media/peng/Samsung/Helios_results/test/images", i);
  //   radiation_model.writeNormDepthImage("visible", "depth", 12, "/media/peng/Samsung/Helios_results/test/depth", i);

  //   // Print progress
  //   if (i % 10 == 0)
  //     printf("Processed %d out of %d samples\n", i, num_samples);
  // }

  // Visualizer
  Visualizer visualizer(1000);
  visualizer.hideWatermark();
  visualizer.setBackgroundColor(RGB::gray);
  visualizer.addSkyDomeByCenter(100, make_vec3(0, 0, 0), 30, "plugins/visualizer/textures/SkyDome_clouds.jpg");
  visualizer.buildContextGeometry(&context);

//   visualizer.colorContextPrimitivesByData("radiation_flux_red");
  // visualizer.setColorbarTitle("flux [W/m^2]" );
  visualizer.disableColorbar();

  visualizer.setLightingModel(Visualizer::LIGHTING_PHONG_SHADOWED);
  visualizer.setLightDirection(sun_dir);
  visualizer.setCameraPosition(camera_position, camera_lookat);
  visualizer.setCameraFieldOfView(camera_properties.HFOV);
 
  visualizer.plotUpdate();
  // visualizer.printWindow("./visualizer_1.jpeg");
  visualizer.plotInteractive();

  return 0;
}
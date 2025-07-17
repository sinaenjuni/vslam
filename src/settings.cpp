#include "settings.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip>

void Settings::print() const
{
  std::cout << "Settings:"
            << "\n";
  std::cout << "3D viewer Information:"
            << "\n";
  std::cout << "  Window Name: " << window_name << "\n";
  std::cout << "  Window Width: " << windowWidth << "\n";
  std::cout << "  Window Height: " << windowHeight << "\n";
  std::cout << "  ViewpointX: " << ViewpointX << "\n";
  std::cout << "  ViewpointY: " << ViewpointY << "\n";
  std::cout << "  ViewpointZ: " << ViewpointZ << "\n";
  std::cout << "  ViewpointF: " << ViewpointF << "\n";

  std::cout << "Sensor Type: " << sensor_type << "\n";
  std::cout << "Camera Information:"
            << "\n";
  std::cout << "  Image Width: " << imgWidth << "\n";
  std::cout << "  Image Height: " << imgHeight << "\n";
  std::cout << "  fx: " << fx << "\n";
  std::cout << "  fy: " << fy << "\n";
  std::cout << "  cx: " << cx << "\n";
  std::cout << "  cy: " << cy << "\n";
  std::cout << "  bf: " << bf << "\n";

  std::cout << "Image and Time Data:"
            << "\n";
  std::cout << "  Image Path: " << imgPath << "\n";
  std::cout << "  Image Right Path: " << imgrPath << "\n";
  std::cout << "  Timestamp Path: " << timestampPath << "\n";
  std::cout << "  Timestamps: ";
  // for (const auto& ts : timestamp) {
  // std::cout << std::fixed << std::setprecision(6) << ts << " ";
  // }
  std::cout << "\n";

  std::cout << "Feature Settings:"
            << "\n";
  std::cout << "  Number of Levels: " << nLevels << "\n";
  std::cout << "  Number of Points: " << nPoints << "\n";
  std::cout << "  Feature Size: " << featureSize << "\n";
  std::cout << "  Scale Factor: " << scaleFactor << "\n";

  std::cout << "  Scale Factors: "
            << "\n";
  for (const auto &factor : scale2Factors)
  {
    std::cout << std::fixed << std::setprecision(6) << factor << " ";
  }
  std::cout << "\n";

  std::cout << "  Inverse Scale Factors: "
            << "\n";
  for (const auto &factor : scale2InvFactors)
  {
    std::cout << std::fixed << std::setprecision(6) << factor << " ";
  }
  std::cout << "\n";

  std::cout << "Optimization Parameters:"
            << "\n";
  // std::cout << "  Chi2 Mono: " << kChi2Mono << "\n";
  // std::cout << "  Chi2 Stereo: " << kChi2Stereo << "\n";
  std::cout << "  Cosine Max Parallax: " << cos_max_parallax << "\n";
  std::cout << "  Scale Consistency Factor: " << scale_consistency_factor
            << "\n";
  std::cout << "  Desired Median Depth: " << desired_median_depth << "\n";
  std::cout << "  Chi Square  Threshold: " << chi_square_threshold << "\n";
  std::cout << "  Use Verbose: " << use_verbose << "\n";
  std::cout << "  Use Robust Kernel: " << use_robust_kernel << "\n";
}

Settings Settings::prase_settings(const std::string &yaml_file_path)
{
  // try {
  YAML::Node yaml_file;
  try
  {
    yaml_file = YAML::LoadFile(yaml_file_path);
  }
  catch (const YAML::Exception &e)
  {
    std::cerr << "Error: YAML file " << e.what() << "\n";
    std::exit(EXIT_FAILURE);
  }
  Settings settings;
  settings.timestampPath = yaml_file["timestamp_path"].as<std::string>();
  std::ifstream file(settings.timestampPath);
  if (!file.is_open())
  {
    std::cerr << "Error: timestemp file cant load."
              << "\n";
    std::exit(EXIT_FAILURE);
  }

  std::string line;
  while (std::getline(file, line))
  {
    // std::cout << std::stod(line) << " " << std::stof(line) << "\n";
    settings.timestamp.push_back(std::stof(line));
  }
  file.close();

  settings.window_name = yaml_file["window_name"].as<std::string>();
  settings.windowWidth = yaml_file["window_width"].as<int>();
  settings.windowHeight = yaml_file["window_height"].as<int>();
  settings.ViewpointX = yaml_file["ViewpointX"].as<int>();
  settings.ViewpointY = yaml_file["ViewpointY"].as<int>();
  settings.ViewpointZ = yaml_file["ViewpointZ"].as<float>();
  settings.ViewpointF = yaml_file["ViewpointF"].as<int>();

  settings.nLevels = yaml_file["n_levels"].as<int>();
  settings.nPoints = yaml_file["n_points"].as<int>();
  settings.featureSize = yaml_file["feature_size"].as<int>();
  settings.scaleFactor = yaml_file["scale_factor"].as<float>();
  settings.scale2Factors.resize(settings.nLevels);
  settings.scale2InvFactors.resize(settings.nLevels);

  for (int i = 0; i < settings.nLevels; i++)
  {
    settings.scale2Factors[i] = std::pow(settings.scaleFactor, i);
    settings.scale2InvFactors[i] = 1 / settings.scale2Factors[i];
  }

  settings.sensor_type = yaml_file["sensor_type"].as<std::string>();
  settings.imgWidth = yaml_file["img_width"].as<int>();
  settings.imgHeight = yaml_file["img_height"].as<int>();

  settings.fx = yaml_file["fx"].as<float>();
  settings.fy = yaml_file["fy"].as<float>();
  settings.cy = yaml_file["cy"].as<float>();
  settings.cx = yaml_file["cx"].as<float>();
  settings.bf = yaml_file["baseline_with_fx"].as<float>();

  settings.k1 = yaml_file["k1"].as<float>();
  settings.k2 = yaml_file["k2"].as<float>();
  settings.p1 = yaml_file["p1"].as<float>();
  settings.p2 = yaml_file["p2"].as<float>();

  settings.imgPath = yaml_file["img_path"].as<std::string>();
  settings.imgrPath = yaml_file["imgr_path"].as<std::string>();

  // settings.kChi2Mono = yaml_file["kChi2Mono"].as<float>();
  // settings.kChi2Stereo = yaml_file["kChi2Stereo"].as<float>();
  settings.cos_max_parallax = yaml_file["cos_max_parallax"].as<float>();
  settings.scale_consistency_factor =
      yaml_file["scale_consistency_factor"].as<float>();
  settings.desired_median_depth = yaml_file["desired_median_depth"].as<int>();

  settings.chi_square_threshold = yaml_file["chi_square_threshold"].as<float>();
  settings.use_verbose = yaml_file["use_verbose"].as<bool>();
  settings.use_robust_kernel = yaml_file["use_robust_kernel"].as<bool>();

  return settings;
};

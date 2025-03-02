#include "settings.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip>

void Settings::print() const
{
  std::cout << "Settings:"
            << "\n";
  std::cout << "Sensor Type: " << sensor_type << "\n";
  std::cout << "Camera Information:"
            << "\n";
  std::cout << "  Width: " << width << "\n";
  std::cout << "  Height: " << height << "\n";
  std::cout << "  fx: " << fx << "\n";
  std::cout << "  fy: " << fy << "\n";
  std::cout << "  cx: " << cx << "\n";
  std::cout << "  cy: " << cy << "\n";
  std::cout << "  bf: " << bf << "\n";

  std::cout << "Image and Time Data:"
            << "\n";
  std::cout << "  Image Path: " << img_path << "\n";
  std::cout << "  Image Right Path: " << imgr_path << "\n";
  std::cout << "  Timestamp Path: " << timestamp_path << "\n";
  std::cout << "  Timestamps: ";
  // for (const auto& ts : timestamp) {
  // std::cout << std::fixed << std::setprecision(6) << ts << " ";
  // }
  std::cout << "\n";

  std::cout << "Feature Settings:"
            << "\n";
  std::cout << "  Number of Levels: " << n_levels << "\n";
  std::cout << "  Number of Points: " << n_points << "\n";
  std::cout << "  Feature Size: " << feature_size << "\n";
  std::cout << "  Scale Factor: " << scale_factor << "\n";
  std::cout << "  Max_tracking_size: " << max_tracking_size << "\n";

  std::cout << "  Scale Factors: "
            << "\n";
  for (const auto &factor : scale2_factors)
  {
    std::cout << std::fixed << std::setprecision(6) << factor << " ";
  }
  std::cout << "\n";

  std::cout << "  Inverse Scale Factors: "
            << "\n";
  for (const auto &factor : scale2inv_factors)
  {
    std::cout << std::fixed << std::setprecision(6) << factor << " ";
  }
  std::cout << "\n";

  std::cout << "Optimization Parameters:"
            << "\n";
  std::cout << "  Chi2 Mono: " << kChi2Mono << "\n";
  std::cout << "  Chi2 Stereo: " << kChi2Stereo << "\n";
  std::cout << "  Cosine Max Parallax: " << cos_max_parallax << "\n";
  std::cout << "  Scale Consistency Factor: " << scale_consistency_factor << "\n";
  std::cout << "  Desired Median Depth: " << desired_median_depth << "\n";
}

Settings prase_settings(const std::string &yaml_file_path)
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
  settings.timestamp_path = yaml_file["timestamp_path"].as<std::string>();
  std::ifstream file(settings.timestamp_path);
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

  settings.n_levels = yaml_file["n_levels"].as<int>();
  settings.n_points = yaml_file["n_points"].as<int>();
  settings.feature_size = yaml_file["feature_size"].as<int>();
  settings.scale_factor = yaml_file["scale_factor"].as<double>();
  settings.scale2_factors.resize(settings.n_levels);
  settings.scale2inv_factors.resize(settings.n_levels);
  settings.max_tracking_size = yaml_file["max_tracking_size"].as<int>();

  for (size_t i = 0; i < settings.n_levels; i++)
  {
    settings.scale2_factors[i] = std::pow(settings.scale_factor, i);
    settings.scale2inv_factors[i] = 1 / settings.scale2_factors[i];
  }

  settings.sensor_type = yaml_file["sensor_type"].as<std::string>();
  settings.width = yaml_file["width"].as<int>();
  settings.height = yaml_file["height"].as<int>();

  settings.fx = yaml_file["fx"].as<float>();
  settings.fy = yaml_file["fy"].as<float>();
  settings.cx = yaml_file["cx"].as<float>();
  settings.cy = yaml_file["cy"].as<float>();
  settings.bf = yaml_file["baseline_with_fx"].as<double>();

  settings.img_path = yaml_file["img_path"].as<std::string>();
  settings.imgr_path = yaml_file["imgr_path"].as<std::string>();

  settings.kChi2Mono = yaml_file["kChi2Mono"].as<double>();
  settings.kChi2Stereo = yaml_file["kChi2Stereo"].as<double>();
  settings.cos_max_parallax = yaml_file["cos_max_parallax"].as<double>();
  settings.scale_consistency_factor = yaml_file["scale_consistency_factor"].as<double>();
  settings.desired_median_depth = yaml_file["desired_median_depth"].as<int>();

  return settings;
};

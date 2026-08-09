#include "grid_utils.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

namespace
{

struct MapYaml
{
  std::string image;
  std::string mode{"trinary"};
  double resolution{0.05};
  std::array<double, 3> origin{0.0, 0.0, 0.0};
  bool negate{false};
  double occupied_thresh{0.65};
  double free_thresh{0.25};
};

// 去掉空白。
std::string trim(const std::string & input)
{
  const auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char c) {
    return std::isspace(c);
  });
  const auto last = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c) {
    return std::isspace(c);
  }).base();
  if (first >= last) {
    return "";
  }
  return std::string(first, last);
}

// 去掉可能包裹路径的引号。
std::string stripQuotes(std::string value)
{
  value = trim(value);
  if (value.size() >= 2) {
    const char first = value.front();
    const char last = value.back();
    if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
      return value.substr(1, value.size() - 2);
    }
  }
  return value;
}

// 去掉行内注释，但保留引号里的 #。
std::string stripInlineComment(const std::string & line)
{
  bool in_single_quote = false;
  bool in_double_quote = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '\'' && !in_double_quote) {
      in_single_quote = !in_single_quote;
    } else if (c == '"' && !in_single_quote) {
      in_double_quote = !in_double_quote;
    } else if (c == '#' && !in_single_quote && !in_double_quote) {
      return line.substr(0, i);
    }
  }
  return line;
}

// 解析 yaml 里的 origin 字段。
std::array<double, 3> parseOrigin(const std::string & value)
{
  std::string cleaned = value;
  cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
  cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
  std::replace(cleaned.begin(), cleaned.end(), ',', ' ');

  std::istringstream stream(cleaned);
  std::array<double, 3> origin{0.0, 0.0, 0.0};
  stream >> origin[0] >> origin[1] >> origin[2];
  if (!stream) {
    throw std::runtime_error("Invalid map origin: " + value);
  }
  return origin;
}

// 读取一份最小化的 map yaml。
MapYaml readMapYaml(const std::string & yaml_path)
{
  std::ifstream input(yaml_path);
  if (!input.is_open()) {
    throw std::runtime_error("Cannot open map yaml: " + yaml_path);
  }

  MapYaml yaml;
  std::string line;
  while (std::getline(input, line)) {
    line = trim(stripInlineComment(line));
    if (line.empty()) {
      continue;
    }

    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }

    const std::string key = trim(line.substr(0, colon));
    const std::string value = stripQuotes(line.substr(colon + 1));

    if (key == "image") {
      yaml.image = value;
    } else if (key == "mode") {
      yaml.mode = value;
    } else if (key == "resolution") {
      yaml.resolution = std::stod(value);
    } else if (key == "origin") {
      yaml.origin = parseOrigin(value);
    } else if (key == "negate") {
      yaml.negate = std::stoi(value) != 0;
    } else if (key == "occupied_thresh") {
      yaml.occupied_thresh = std::stod(value);
    } else if (key == "free_thresh") {
      yaml.free_thresh = std::stod(value);
    }
  }

  if (yaml.image.empty()) {
    throw std::runtime_error("Map yaml does not contain an image field: " + yaml_path);
  }
  if (yaml.resolution <= 0.0) {
    throw std::runtime_error("Map resolution must be positive: " + yaml_path);
  }
  return yaml;
}

// 读 PGM 头里的下一个 token。
std::string readPgmToken(std::istream & input)
{
  char c = '\0';
  while (input.get(c)) {
    if (std::isspace(static_cast<unsigned char>(c))) {
      continue;
    }
    if (c == '#') {
      input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      continue;
    }
    input.unget();
    break;
  }

  std::string token;
  input >> token;
  return token;
}

// 读 PGM 图像数据。
std::vector<unsigned int> readPgm(
  const std::string & image_path, unsigned int & width, unsigned int & height,
  unsigned int & max_value)
{
  std::ifstream input(image_path, std::ios::binary);
  if (!input.is_open()) {
    throw std::runtime_error("Cannot open map image: " + image_path);
  }

  const std::string magic = readPgmToken(input);
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error("Only PGM P5/P2 images are supported: " + image_path);
  }

  width = static_cast<unsigned int>(std::stoul(readPgmToken(input)));
  height = static_cast<unsigned int>(std::stoul(readPgmToken(input)));
  max_value = static_cast<unsigned int>(std::stoul(readPgmToken(input)));
  if (width == 0 || height == 0 || max_value == 0) {
    throw std::runtime_error("Invalid PGM header: " + image_path);
  }

  std::vector<unsigned int> pixels(width * height, 0);
  if (magic == "P5") {
    input.get();
    std::vector<unsigned char> raw(width * height, 0);
    input.read(reinterpret_cast<char *>(raw.data()), static_cast<std::streamsize>(raw.size()));
    if (input.gcount() != static_cast<std::streamsize>(raw.size())) {
      throw std::runtime_error("PGM file ended before all pixels were read: " + image_path);
    }
    for (std::size_t i = 0; i < raw.size(); ++i) {
      pixels[i] = raw[i];
    }
  } else {
    for (std::size_t i = 0; i < pixels.size(); ++i) {
      input >> pixels[i];
      if (!input) {
        throw std::runtime_error("PGM file ended before all text pixels were read: " + image_path);
      }
    }
  }

  return pixels;
}

}  // namespace

namespace navigation2
{

class RmMapServer : public rclcpp::Node
{
public:
  explicit RmMapServer(const rclcpp::NodeOptions & options)
  : Node("rm_map_server", options)
  {
    yaml_filename_ = declare_parameter<std::string>("yaml_filename", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    publish_frequency_ = declare_parameter<double>("publish_frequency", 1.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, qos);
    reload_srv_ = create_service<std_srvs::srv::Empty>(
      "~/reload_map",
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>) {
        loadAndPublish();
      });

    // 按需定时重发地图，方便晚启动的订阅者接上。
    if (publish_frequency_ > 0.0) {
      const auto period =
        std::chrono::duration<double>(1.0 / std::max(0.1, publish_frequency_));
      publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {
          if (has_map_) {
            map_pub_->publish(map_);
          }
        });
    }

    loadAndPublish();
  }

private:
  void loadAndPublish()
  {
    get_parameter("yaml_filename", yaml_filename_);
    if (yaml_filename_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "No yaml_filename was provided. rm_map_server will wait until the parameter is set.");
      has_map_ = false;
      return;
    }

    try {
      map_ = loadMap(yaml_filename_);
      has_map_ = true;
      map_pub_->publish(map_);
      RCLCPP_INFO(
        get_logger(), "Loaded map %s (%ux%u, %.3f m/cell) and published on %s",
        yaml_filename_.c_str(), map_.info.width, map_.info.height, map_.info.resolution,
        map_topic_.c_str());
    } catch (const std::exception & ex) {
      has_map_ = false;
      RCLCPP_ERROR(get_logger(), "Failed to load map: %s", ex.what());
    }
  }

  nav_msgs::msg::OccupancyGrid loadMap(const std::string & yaml_path)
  {
    const auto yaml = readMapYaml(yaml_path);
    const std::filesystem::path yaml_dir = std::filesystem::path(yaml_path).parent_path();
    std::filesystem::path image_path(yaml.image);
    if (image_path.is_relative()) {
      image_path = yaml_dir / image_path;
    }

    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int max_value = 0;
    const auto pixels = readPgm(image_path.string(), width, height, max_value);

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = frame_id_;
    grid.header.stamp = now();
    grid.info.map_load_time = now();
    grid.info.resolution = static_cast<float>(yaml.resolution);
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = yaml.origin[0];
    grid.info.origin.position.y = yaml.origin[1];
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation = quaternionFromYaw(yaml.origin[2]);
    grid.data.assign(static_cast<std::size_t>(width) * height, -1);

    for (unsigned int y = 0; y < height; ++y) {
      for (unsigned int x = 0; x < width; ++x) {
        const auto image_index =
          static_cast<std::size_t>(height - y - 1) * width + static_cast<std::size_t>(x);
        const auto map_index = static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x);
        const double normalized =
          static_cast<double>(pixels[image_index]) / static_cast<double>(max_value);
        const double occupancy = yaml.negate ? normalized : 1.0 - normalized;

        if (occupancy > yaml.occupied_thresh) {
          grid.data[map_index] = 100;
        } else if (occupancy < yaml.free_thresh) {
          grid.data[map_index] = 0;
        } else {
          grid.data[map_index] = -1;
        }
      }
    }

    return grid;
  }

  std::string yaml_filename_;
  std::string frame_id_;
  std::string map_topic_;
  double publish_frequency_{1.0};
  bool has_map_{false};
  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reload_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace navigation2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmMapServer)

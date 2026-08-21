#include "robot_navigation/zone_filter_masks.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

namespace robot_navigation {
namespace {

class ZoneFilterMasksTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_dir_ = std::filesystem::temp_directory_path() /
                ("robot_zone_filter_masks_test_" + std::to_string(::getpid()));
    std::filesystem::remove_all(test_dir_);
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override { std::filesystem::remove_all(test_dir_); }

  std::filesystem::path WritePgm(const std::string& name, const int width, const int height) const {
    const auto path = test_dir_ / name;
    std::ofstream output(path);
    output << "P2\n";
    output << "# test map\n";
    output << width << " " << height << "\n";
    output << "255\n";
    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col) {
        output << "0";
        if (col + 1 < width) {
          output << " ";
        }
      }
      output << "\n";
    }
    return path;
  }

  std::filesystem::path WriteMapYaml() const {
    WritePgm("test_map.pgm", 4, 4);
    const auto path = test_dir_ / "test_map.yaml";
    std::ofstream output(path);
    output << "image: test_map.pgm\n";
    output << "mode: trinary\n";
    output << "resolution: 1.0\n";
    output << "origin: [0.0, 0.0, 0.0]\n";
    output << "negate: 0\n";
    output << "occupied_thresh: 0.65\n";
    output << "free_thresh: 0.25\n";
    return path;
  }

  std::filesystem::path test_dir_;
};

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream input(path);
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

}  // namespace

TEST_F(ZoneFilterMasksTest, LoadOccupancyMapInfo_ValidMapYaml_LoadsImageSizeAndOrigin) {
  const auto map_yaml = WriteMapYaml();

  const auto info = LoadOccupancyMapInfo(map_yaml);

  EXPECT_EQ(info.map_name, "test_map");
  EXPECT_EQ(info.width, 4);
  EXPECT_EQ(info.height, 4);
  EXPECT_DOUBLE_EQ(info.resolution, 1.0);
  EXPECT_DOUBLE_EQ(info.origin_x, 0.0);
  EXPECT_DOUBLE_EQ(info.origin_y, 0.0);
}

TEST_F(ZoneFilterMasksTest, BuildZoneFilterMaskGrid_Zones_RasterizesKeepoutAndSpeedMasks) {
  const auto info = LoadOccupancyMapInfo(WriteMapYaml());
  MapZone keepout;
  keepout.zone_id = "keepout";
  keepout.map_name = "test_map";
  keepout.type = "keepout";
  keepout.polygon_x = {1.0, 3.0, 3.0, 1.0};
  keepout.polygon_y = {1.0, 1.0, 3.0, 3.0};
  MapZone speed;
  speed.zone_id = "slow";
  speed.map_name = "test_map";
  speed.type = "speed_limit";
  speed.speed_limit_mps = 0.25;
  speed.polygon_x = {0.0, 2.0, 2.0, 0.0};
  speed.polygon_y = {0.0, 0.0, 2.0, 2.0};

  const auto grid = BuildZoneFilterMaskGrid(info, {keepout, speed});

  ASSERT_EQ(grid.keepout_values.size(), 16U);
  ASSERT_EQ(grid.speed_values.size(), 16U);
  EXPECT_EQ(grid.keepout_cells, 4);
  EXPECT_EQ(grid.speed_limit_cells, 4);
  EXPECT_EQ(grid.keepout_values[1 * 4 + 1], 100);
  EXPECT_EQ(grid.keepout_values[2 * 4 + 2], 100);
  EXPECT_EQ(grid.speed_values[2 * 4 + 0], 25);
  EXPECT_EQ(grid.speed_values[3 * 4 + 1], 25);
}

TEST_F(ZoneFilterMasksTest, BuildZoneFilterMaskGrid_DisabledZone_IgnoresZone) {
  const auto info = LoadOccupancyMapInfo(WriteMapYaml());
  MapZone keepout;
  keepout.zone_id = "disabled_keepout";
  keepout.map_name = "test_map";
  keepout.type = "keepout";
  keepout.enabled = false;
  keepout.polygon_x = {0.0, 4.0, 4.0, 0.0};
  keepout.polygon_y = {0.0, 0.0, 4.0, 4.0};

  const auto grid = BuildZoneFilterMaskGrid(info, {keepout});

  EXPECT_EQ(grid.keepout_cells, 0);
  EXPECT_EQ(grid.speed_limit_cells, 0);
}

TEST_F(ZoneFilterMasksTest, WriteZoneFilterMaskAssets_ValidGrid_WritesMaskYamlAndProfile) {
  const auto info = LoadOccupancyMapInfo(WriteMapYaml());
  MapZone speed;
  speed.zone_id = "slow";
  speed.map_name = "test_map";
  speed.type = "speed_limit";
  speed.speed_limit_mps = 0.35;
  speed.polygon_x = {0.0, 2.0, 2.0, 0.0};
  speed.polygon_y = {0.0, 0.0, 2.0, 2.0};
  const auto grid = BuildZoneFilterMaskGrid(info, {speed});

  const auto summary = WriteZoneFilterMaskAssets(info, grid, test_dir_ / "filters");

  EXPECT_TRUE(std::filesystem::exists(summary.keepout_mask_yaml));
  EXPECT_TRUE(std::filesystem::exists(summary.speed_mask_yaml));
  EXPECT_TRUE(std::filesystem::exists(summary.nav2_profile_yaml));
  const auto profile = ReadFile(summary.nav2_profile_yaml);
  EXPECT_NE(profile.find("nav2_costmap_2d::KeepoutFilter"), std::string::npos);
  EXPECT_NE(profile.find("nav2_costmap_2d::SpeedFilter"), std::string::npos);
  EXPECT_NE(profile.find("type: 2"), std::string::npos);
  EXPECT_NE(profile.find("multiplier: 0.01"), std::string::npos);
}

}  // namespace robot_navigation

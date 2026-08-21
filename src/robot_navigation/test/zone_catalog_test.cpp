#include "robot_navigation/zone_catalog.hpp"

#include <filesystem>
#include <fstream>
#include <unistd.h>

#include <gtest/gtest.h>

namespace robot_navigation {
namespace {

class ZoneCatalogTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_dir_ = std::filesystem::temp_directory_path() /
                ("robot_zone_catalog_test_" + std::to_string(::getpid()));
    std::filesystem::remove_all(test_dir_);
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override { std::filesystem::remove_all(test_dir_); }

  std::filesystem::path WriteZones(const std::string& content) const {
    const auto path = test_dir_ / "zones.yaml";
    std::ofstream out(path);
    out << content;
    return path;
  }

  std::filesystem::path test_dir_;
};

}  // namespace

TEST_F(ZoneCatalogTest, LoadMapZones_ValidZones_ReturnsSortedZones) {
  const auto path = WriteZones(
      "zones:\n"
      "  - zone_id: slow_a\n"
      "    map_name: indoor_room\n"
      "    type: speed_limit\n"
      "    frame_id: map\n"
      "    enabled: true\n"
      "    speed_limit_mps: 0.2\n"
      "    rule: slow\n"
      "    polygon:\n"
      "      - [0.0, 0.0]\n"
      "      - [1.0, 0.0]\n"
      "      - [1.0, 1.0]\n"
      "  - zone_id: keepout_b\n"
      "    map_name: indoor_room\n"
      "    type: keepout\n"
      "    frame_id: map\n"
      "    enabled: true\n"
      "    speed_limit_mps: 0.0\n"
      "    rule: blocked\n"
      "    polygon:\n"
      "      - [2.0, 2.0]\n"
      "      - [3.0, 2.0]\n"
      "      - [3.0, 3.0]\n");

  const auto zones = LoadMapZones(path);

  ASSERT_EQ(zones.size(), 2U);
  EXPECT_EQ(zones[0].zone_id, "keepout_b");
  EXPECT_EQ(zones[1].zone_id, "slow_a");
  EXPECT_DOUBLE_EQ(zones[1].speed_limit_mps, 0.2);
}

TEST_F(ZoneCatalogTest, ForMap_DisabledExcludedByDefault_ReturnsOnlyEnabledZones) {
  const auto path = WriteZones(
      "zones:\n"
      "  - zone_id: enabled_zone\n"
      "    map_name: indoor_room\n"
      "    type: speed_limit\n"
      "    frame_id: map\n"
      "    enabled: true\n"
      "    speed_limit_mps: 0.3\n"
      "    rule: enabled\n"
      "    polygon:\n"
      "      - [0.0, 0.0]\n"
      "      - [1.0, 0.0]\n"
      "      - [1.0, 1.0]\n"
      "  - zone_id: disabled_zone\n"
      "    map_name: indoor_room\n"
      "    type: preferred_lane\n"
      "    frame_id: map\n"
      "    enabled: false\n"
      "    speed_limit_mps: 0.0\n"
      "    rule: disabled\n"
      "    polygon:\n"
      "      - [0.0, 0.0]\n"
      "      - [1.0, 0.0]\n"
      "      - [1.0, 1.0]\n");
  ZoneCatalog catalog(path);

  EXPECT_EQ(catalog.ForMap("indoor_room", false).size(), 1U);
  EXPECT_EQ(catalog.ForMap("indoor_room", true).size(), 2U);
}

TEST_F(ZoneCatalogTest, EvaluatePoint_InsideSpeedLimitZone_ReducesSpeed) {
  const auto path = WriteZones(
      "zones:\n"
      "  - zone_id: slow_zone\n"
      "    map_name: indoor_room\n"
      "    type: speed_limit\n"
      "    frame_id: map\n"
      "    enabled: true\n"
      "    speed_limit_mps: 0.25\n"
      "    rule: slow\n"
      "    polygon:\n"
      "      - [0.0, 0.0]\n"
      "      - [2.0, 0.0]\n"
      "      - [2.0, 2.0]\n"
      "      - [0.0, 2.0]\n");
  ZoneCatalog catalog(path);

  const auto evaluation = catalog.EvaluatePoint("indoor_room", 1.0, 1.0, 0.6);

  EXPECT_TRUE(evaluation.allowed);
  EXPECT_DOUBLE_EQ(evaluation.speed_limit_mps, 0.25);
  ASSERT_EQ(evaluation.matched_zones.size(), 1U);
  EXPECT_EQ(evaluation.matched_zones[0].zone_id, "slow_zone");
}

TEST_F(ZoneCatalogTest, EvaluatePoint_InsideKeepoutZone_DisallowsPoint) {
  const auto path = WriteZones(
      "zones:\n"
      "  - zone_id: keepout_zone\n"
      "    map_name: indoor_room\n"
      "    type: keepout\n"
      "    frame_id: map\n"
      "    enabled: true\n"
      "    speed_limit_mps: 0.0\n"
      "    rule: blocked\n"
      "    polygon:\n"
      "      - [-1.0, -1.0]\n"
      "      - [1.0, -1.0]\n"
      "      - [1.0, 1.0]\n"
      "      - [-1.0, 1.0]\n");
  ZoneCatalog catalog(path);

  const auto evaluation = catalog.EvaluatePoint("indoor_room", 0.0, 0.0, 0.6);

  EXPECT_FALSE(evaluation.allowed);
  EXPECT_DOUBLE_EQ(evaluation.speed_limit_mps, 0.6);
  ASSERT_EQ(evaluation.matched_zones.size(), 1U);
  EXPECT_EQ(evaluation.matched_zones[0].type, "keepout");
}

TEST_F(ZoneCatalogTest, LoadMapZones_InvalidPolygon_Throws) {
  const auto path = WriteZones(
      "zones:\n"
      "  - zone_id: broken_zone\n"
      "    map_name: indoor_room\n"
      "    type: speed_limit\n"
      "    frame_id: map\n"
      "    enabled: true\n"
      "    speed_limit_mps: 0.2\n"
      "    rule: broken\n"
      "    polygon:\n"
      "      - [0.0, 0.0]\n"
      "      - [1.0, 0.0]\n");

  EXPECT_THROW(LoadMapZones(path), std::runtime_error);
}

}  // namespace robot_navigation

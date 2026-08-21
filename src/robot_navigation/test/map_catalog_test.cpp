#include "robot_navigation/map_catalog.hpp"

#include <filesystem>
#include <fstream>
#include <unistd.h>

#include <gtest/gtest.h>

namespace robot_navigation {
namespace {

class MapCatalogTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_dir_ = std::filesystem::temp_directory_path() /
                ("robot_map_catalog_test_" + std::to_string(::getpid()));
    std::filesystem::remove_all(test_dir_);
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override { std::filesystem::remove_all(test_dir_); }

  void WriteFile(const std::string& name, const std::string& content) const {
    std::ofstream out(test_dir_ / name);
    out << content;
  }

  std::filesystem::path test_dir_;
};

}  // namespace

TEST_F(MapCatalogTest, Scan_ValidMap_ReturnsMapInfo) {
  WriteFile("lab.pgm", "P2\n1 1\n255\n0\n");
  WriteFile(
      "lab.yaml",
      "image: lab.pgm\nmode: trinary\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n");
  MapCatalog catalog(test_dir_);

  const auto maps = catalog.Scan();

  ASSERT_EQ(maps.size(), 1U);
  EXPECT_EQ(maps[0].name, "lab");
  EXPECT_TRUE(maps[0].valid);
  EXPECT_DOUBLE_EQ(maps[0].resolution, 0.05);
}

TEST_F(MapCatalogTest, Scan_InvalidMapMissingImageFile_IgnoresMap) {
  WriteFile(
      "broken.yaml",
      "image: missing.pgm\nmode: trinary\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n");
  MapCatalog catalog(test_dir_);

  EXPECT_TRUE(catalog.Scan().empty());
}

TEST_F(MapCatalogTest, FindByName_ExistingMap_ReturnsMap) {
  WriteFile("a.pgm", "P2\n1 1\n255\n0\n");
  WriteFile("b.pgm", "P2\n1 1\n255\n0\n");
  WriteFile("a.yaml", "image: a.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n");
  WriteFile("b.yaml", "image: b.pgm\nresolution: 0.2\norigin: [0, 0, 0]\n");
  MapCatalog catalog(test_dir_);

  const auto map = catalog.FindByName("b");

  ASSERT_TRUE(map.has_value());
  EXPECT_EQ(map->name, "b");
  EXPECT_DOUBLE_EQ(map->resolution, 0.2);
}

TEST_F(MapCatalogTest, FindByName_MissingMap_ReturnsNullopt) {
  MapCatalog catalog(test_dir_);

  EXPECT_FALSE(catalog.FindByName("missing").has_value());
}

}  // namespace robot_navigation

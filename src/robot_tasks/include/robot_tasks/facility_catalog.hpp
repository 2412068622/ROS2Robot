#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace robot_tasks {

struct FacilityResource {
  std::string id;
  std::string type;
  std::string station_id;
  bool enabled = true;
  bool exclusive = true;
};

struct FacilityCatalog {
  std::vector<FacilityResource> resources;
};

std::optional<FacilityCatalog> LoadFacilityCatalog(const std::filesystem::path& path);
const FacilityResource* FindFacilityResource(
    const FacilityCatalog& catalog, const std::string& resource_id);
const FacilityResource* FindFirstAvailableResource(
    const FacilityCatalog& catalog, const std::string& resource_type,
    const std::vector<std::string>& unavailable_resource_ids);
bool ValidateFacilityCatalog(const FacilityCatalog& catalog, std::string* message);

}  // namespace robot_tasks

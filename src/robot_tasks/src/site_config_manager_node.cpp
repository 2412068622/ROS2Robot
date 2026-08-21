#include <filesystem>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces_site/srv/activate_site_config.hpp"
#include "robot_interfaces_site/srv/rollback_site_config.hpp"
#include "robot_interfaces_site/srv/validate_site_config.hpp"
#include "robot_tasks/business_order_catalog.hpp"
#include "robot_tasks/dock_catalog.hpp"
#include "robot_tasks/facility_catalog.hpp"
#include "robot_tasks/fleet_catalog.hpp"
#include "robot_tasks/scenario_catalog.hpp"
#include "robot_tasks/station_catalog.hpp"

namespace {

struct BundlePaths {
  std::filesystem::path root;
  std::filesystem::path stations;
  std::filesystem::path facilities;
  std::filesystem::path fleet;
  std::filesystem::path scenarios;
  std::filesystem::path business_orders;
  std::filesystem::path docks;
};

struct ValidationResult {
  bool success = false;
  std::string message;
  std::string version_id;
  std::vector<std::string> checked_files;
  std::vector<std::string> errors;
  std::string diff_report;
};

BundlePaths ResolveBundlePaths(const std::filesystem::path& root) {
  return BundlePaths{
      root,
      root / "stations.yaml",
      root / "facilities.yaml",
      root / "fleet.yaml",
      root / "scenarios.yaml",
      root / "business_orders.yaml",
      root / "docks.yaml"};
}

void RequireFile(
    const std::filesystem::path& path, std::vector<std::string>* checked_files,
    std::vector<std::string>* errors) {
  checked_files->push_back(path.string());
  if (!std::filesystem::exists(path)) {
    errors->push_back("missing file: " + path.string());
  }
}

std::string VersionFromRequest(const std::string& request_version, const std::filesystem::path& root) {
  if (!request_version.empty()) {
    return request_version;
  }
  return root.filename().empty() ? "site_config" : root.filename().string();
}

std::string MakeDiffReport(
    const std::string& active_version, const std::string& next_version,
    const std::string& active_dir, const std::string& next_dir) {
  std::ostringstream out;
  out << "active_version=" << (active_version.empty() ? "<none>" : active_version)
      << "; next_version=" << next_version
      << "; active_bundle=" << (active_dir.empty() ? "<none>" : active_dir)
      << "; next_bundle=" << next_dir;
  return out.str();
}

}  // namespace

namespace v2_site_srv = robot_interfaces_site::srv;

class SiteConfigManagerNode final : public rclcpp::Node {
 public:
  SiteConfigManagerNode() : Node("site_config_manager_node") {
    v2_validate_srv_ = create_service<v2_site_srv::ValidateSiteConfig>(
        "/v2/validate_site_config",
        [this](const std::shared_ptr<v2_site_srv::ValidateSiteConfig::Request> request,
               std::shared_ptr<v2_site_srv::ValidateSiteConfig::Response> response) {
          ValidateSiteConfig(request, response);
        });
    v2_activate_srv_ = create_service<v2_site_srv::ActivateSiteConfig>(
        "/v2/activate_site_config",
        [this](const std::shared_ptr<v2_site_srv::ActivateSiteConfig::Request> request,
               std::shared_ptr<v2_site_srv::ActivateSiteConfig::Response> response) {
          ActivateSiteConfig(request, response);
        });
    v2_rollback_srv_ = create_service<v2_site_srv::RollbackSiteConfig>(
        "/v2/rollback_site_config",
        [this](const std::shared_ptr<v2_site_srv::RollbackSiteConfig::Request>,
               std::shared_ptr<v2_site_srv::RollbackSiteConfig::Response> response) {
          RollbackSiteConfig(response);
        });
  }

 private:
  ValidationResult ValidateBundle(
      const std::filesystem::path& bundle_dir, const std::string& requested_version) const {
    ValidationResult result;
    result.version_id = VersionFromRequest(requested_version, bundle_dir);
    const auto paths = ResolveBundlePaths(bundle_dir);
    if (bundle_dir.empty() || !std::filesystem::is_directory(bundle_dir)) {
      result.errors.push_back("bundle_dir is not a directory: " + bundle_dir.string());
      result.message = "site config validation failed";
      result.diff_report = MakeDiffReport(
          active_version_id_, result.version_id, active_bundle_dir_, bundle_dir.string());
      return result;
    }

    RequireFile(paths.stations, &result.checked_files, &result.errors);
    RequireFile(paths.facilities, &result.checked_files, &result.errors);
    RequireFile(paths.fleet, &result.checked_files, &result.errors);
    RequireFile(paths.scenarios, &result.checked_files, &result.errors);
    RequireFile(paths.business_orders, &result.checked_files, &result.errors);
    RequireFile(paths.docks, &result.checked_files, &result.errors);
    if (!result.errors.empty()) {
      result.message = "site config validation failed";
      result.diff_report = MakeDiffReport(
          active_version_id_, result.version_id, active_bundle_dir_, bundle_dir.string());
      return result;
    }

    const auto stations = robot_tasks::LoadStationCatalog(paths.stations);
    const auto facilities = robot_tasks::LoadFacilityCatalog(paths.facilities);
    const auto fleet = robot_tasks::LoadFleetCatalog(paths.fleet);
    const auto scenarios = robot_tasks::LoadScenarioCatalog(paths.scenarios);
    const auto business_orders = robot_tasks::LoadBusinessOrderCatalog(paths.business_orders);
    const auto docks = robot_tasks::LoadDockCatalog(paths.docks);

    std::string message;
    if (!stations.has_value() || !robot_tasks::ValidateStationCatalog(*stations, &message)) {
      result.errors.push_back("invalid stations.yaml: " + message);
    }
    if (!facilities.has_value() || !robot_tasks::ValidateFacilityCatalog(*facilities, &message)) {
      result.errors.push_back("invalid facilities.yaml: " + message);
    }
    if (!fleet.has_value() || !robot_tasks::ValidateFleetCatalog(*fleet, &message)) {
      result.errors.push_back("invalid fleet.yaml: " + message);
    }
    if (!scenarios.has_value() || !robot_tasks::ValidateScenarioCatalog(*scenarios, &message)) {
      result.errors.push_back("invalid scenarios.yaml: " + message);
    }
    if (!business_orders.has_value()) {
      result.errors.push_back("invalid business_orders.yaml");
    }
    if (!docks.has_value() || !robot_tasks::ValidateDockCatalog(*docks, &message)) {
      result.errors.push_back("invalid docks.yaml: " + message);
    }
    if (result.errors.empty()) {
      ValidateCrossReferences(*stations, *facilities, *fleet, *scenarios, *business_orders, *docks,
                              &result.errors);
    }

    result.success = result.errors.empty();
    result.message = result.success ? "site config validation passed" : "site config validation failed";
    result.diff_report = MakeDiffReport(
        active_version_id_, result.version_id, active_bundle_dir_, bundle_dir.string());
    return result;
  }

  void ValidateCrossReferences(
      const robot_tasks::StationCatalog& stations, const robot_tasks::FacilityCatalog& facilities,
      const robot_tasks::FleetCatalog& fleet, const robot_tasks::ScenarioCatalog& scenarios,
      const robot_tasks::BusinessOrderCatalog& business_orders,
      const robot_tasks::DockCatalog& docks, std::vector<std::string>* errors) const {
    for (const auto& resource : facilities.resources) {
      if (!resource.station_id.empty() && robot_tasks::FindStation(stations, resource.station_id) == nullptr) {
        errors->push_back("facility " + resource.id + " references unknown station: " + resource.station_id);
      }
    }
    for (const auto& robot : fleet.robots) {
      if (!robot.current_station_id.empty() &&
          robot_tasks::FindStation(stations, robot.current_station_id) == nullptr) {
        errors->push_back("robot " + robot.id + " references unknown station: " + robot.current_station_id);
      }
    }
    for (const auto& task : scenarios.tasks) {
      if (!task.pickup_station_id.empty() &&
          robot_tasks::FindStation(stations, task.pickup_station_id) == nullptr) {
        errors->push_back("scenario task " + task.task_id + " references unknown pickup station: " +
                          task.pickup_station_id);
      }
      if (!task.dropoff_station_id.empty() &&
          robot_tasks::FindStation(stations, task.dropoff_station_id) == nullptr) {
        errors->push_back("scenario task " + task.task_id + " references unknown dropoff station: " +
                          task.dropoff_station_id);
      }
      for (const auto& station_id : task.station_sequence_ids) {
        if (robot_tasks::FindStation(stations, station_id) == nullptr) {
          errors->push_back("scenario task " + task.task_id + " references unknown sequence station: " +
                            station_id);
        }
      }
      if (!task.resource_id.empty() &&
          robot_tasks::FindFacilityResource(facilities, task.resource_id) == nullptr) {
        errors->push_back("scenario task " + task.task_id + " references unknown facility resource: " +
                          task.resource_id);
      }
    }
    for (const auto& order_type : business_orders.order_types) {
      if (robot_tasks::FindScenarioWorkflow(
              scenarios, order_type.scenario_id, order_type.workflow_id) == nullptr) {
        errors->push_back("business order type " + order_type.business_type +
                          " references unknown workflow: " + order_type.scenario_id + "/" +
                          order_type.workflow_id);
      }
    }
    for (const auto& dock : docks.docks) {
      if (robot_tasks::FindStation(stations, dock.station_id) == nullptr) {
        errors->push_back("dock " + dock.id + " references unknown station: " + dock.station_id);
      }
      if (robot_tasks::FindStation(stations, dock.approach_station_id) == nullptr) {
        errors->push_back("dock " + dock.id + " references unknown approach station: " +
                          dock.approach_station_id);
      }
      if (!dock.charger_resource_id.empty() &&
          robot_tasks::FindFacilityResource(facilities, dock.charger_resource_id) == nullptr) {
        errors->push_back("dock " + dock.id + " references unknown charger resource: " +
                          dock.charger_resource_id);
      }
    }
  }

  template <typename RequestT, typename ResponseT>
  void ValidateSiteConfig(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const auto validation = ValidateBundle(request->bundle_dir, request->version_id);
    response->success = validation.success;
    response->message = validation.message;
    response->version_id = validation.version_id;
    response->checked_files = validation.checked_files;
    response->errors = validation.errors;
    response->diff_report = validation.diff_report;
  }

  template <typename RequestT, typename ResponseT>
  void ActivateSiteConfig(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const auto validation = ValidateBundle(request->bundle_dir, request->version_id);
    if (request->require_valid && !validation.success) {
      response->success = false;
      response->message = validation.message;
      response->active_version_id = active_version_id_;
      response->active_bundle_dir = active_bundle_dir_;
      response->previous_version_id = previous_version_id_;
      response->diff_report = validation.diff_report;
      return;
    }
    previous_version_id_ = active_version_id_;
    previous_bundle_dir_ = active_bundle_dir_;
    active_version_id_ = validation.version_id;
    active_bundle_dir_ = request->bundle_dir;
    response->success = true;
    response->message = validation.success ? "site config activated" : "site config activated without validation";
    response->active_version_id = active_version_id_;
    response->active_bundle_dir = active_bundle_dir_;
    response->previous_version_id = previous_version_id_;
    response->diff_report = validation.diff_report;
  }

  template <typename ResponseT>
  void RollbackSiteConfig(const std::shared_ptr<ResponseT>& response) {
    if (previous_version_id_.empty() || previous_bundle_dir_.empty()) {
      response->success = false;
      response->message = "no previous site config to rollback";
      response->active_version_id = active_version_id_;
      response->active_bundle_dir = active_bundle_dir_;
      return;
    }
    const auto rolled_back_from = active_version_id_;
    active_version_id_ = previous_version_id_;
    active_bundle_dir_ = previous_bundle_dir_;
    previous_version_id_.clear();
    previous_bundle_dir_.clear();
    response->success = true;
    response->message = "site config rollback complete";
    response->active_version_id = active_version_id_;
    response->active_bundle_dir = active_bundle_dir_;
    response->rolled_back_from_version_id = rolled_back_from;
  }

  std::string active_version_id_;
  std::string active_bundle_dir_;
  std::string previous_version_id_;
  std::string previous_bundle_dir_;
  rclcpp::Service<v2_site_srv::ValidateSiteConfig>::SharedPtr v2_validate_srv_;
  rclcpp::Service<v2_site_srv::ActivateSiteConfig>::SharedPtr v2_activate_srv_;
  rclcpp::Service<v2_site_srv::RollbackSiteConfig>::SharedPtr v2_rollback_srv_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SiteConfigManagerNode>());
  rclcpp::shutdown();
  return 0;
}

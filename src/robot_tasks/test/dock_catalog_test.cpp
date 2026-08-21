#include "robot_tasks/dock_catalog.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

TEST(DockCatalogTest, BuildDockStateProjection_UsesReservationWhenNoRuntimeState) {
  const Dock dock{"dock_a", "dock_station", "dock_approach", "charger_a", true};
  DockingStateMap dock_states;
  FacilityReservationMap reservations;
  reservations["charger_a"] = ResourceReservation{"mission_a", "mission_a", "CHARGING", true};

  const auto projection = BuildDockStateProjection(dock, dock_states, reservations);

  EXPECT_EQ(projection.dock_id, "dock_a");
  EXPECT_EQ(projection.station_id, "dock_station");
  EXPECT_EQ(projection.approach_station_id, "dock_approach");
  EXPECT_EQ(projection.charger_resource_id, "charger_a");
  EXPECT_TRUE(projection.enabled);
  EXPECT_FALSE(projection.available);
  EXPECT_EQ(projection.state, "CHARGING");
  EXPECT_EQ(projection.mission_id, "mission_a");
}

TEST(DockCatalogTest, BuildDockStateProjection_RuntimeStateOverridesReservation) {
  const Dock dock{"dock_a", "dock_station", "dock_approach", "charger_a", true};
  DockingStateMap dock_states;
  dock_states["dock_a"] = DockingRuntimeState{"dock_a", "mission_runtime", "APPROACHING", true, 1};
  FacilityReservationMap reservations;
  reservations["charger_a"] = ResourceReservation{"mission_a", "mission_a", "CHARGING", true};

  const auto projection = BuildDockStateProjection(dock, dock_states, reservations);

  EXPECT_FALSE(projection.available);
  EXPECT_EQ(projection.state, "APPROACHING");
  EXPECT_EQ(projection.mission_id, "mission_runtime");
}

TEST(DockCatalogTest, BuildDockListProjection_FiltersDisabledDocks) {
  DockCatalog catalog;
  catalog.docks.push_back(Dock{"dock_a", "station_a", "approach_a", "charger_a", true});
  catalog.docks.push_back(Dock{"dock_b", "station_b", "approach_b", "charger_b", false});
  DockingStateMap dock_states;
  FacilityReservationMap reservations;

  const auto enabled_only = BuildDockListProjection(catalog, dock_states, reservations, false);
  const auto include_disabled = BuildDockListProjection(catalog, dock_states, reservations, true);

  EXPECT_EQ(enabled_only.dock_ids, (std::vector<std::string>{"dock_a"}));
  EXPECT_EQ(enabled_only.available, (std::vector<bool>{true}));
  EXPECT_EQ(enabled_only.states, (std::vector<std::string>{"AVAILABLE"}));
  EXPECT_EQ(enabled_only.message, "loaded 1 dock(s)");

  EXPECT_EQ(include_disabled.dock_ids, (std::vector<std::string>{"dock_a", "dock_b"}));
  EXPECT_EQ(include_disabled.available, (std::vector<bool>{true, false}));
  EXPECT_EQ(include_disabled.states, (std::vector<std::string>{"AVAILABLE", "DISABLED"}));
  EXPECT_EQ(include_disabled.message, "loaded 2 dock(s)");
}

}  // namespace
}  // namespace robot_tasks

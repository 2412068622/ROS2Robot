#pragma once

#include <string>

#include "robot_tasks/docking_workflow.hpp"

namespace robot_tasks {

struct DockingBehaviorTreeInput {
  DockingRuntimeState dock_state;
  std::string resource_id = "charger";
};

struct DockingBehaviorTreeResult {
  bool success = false;
  std::string branch;
  DockingRuntimeState dock_state;
  ResourceReservation reservation;
  DockingTransition transition;
};

std::string DefaultDockingBehaviorTreeXml();
DockingBehaviorTreeResult TickDockingBehaviorTree(const DockingBehaviorTreeInput& input);

}  // namespace robot_tasks

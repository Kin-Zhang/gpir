/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <memory>

#include "planning_core/simulation/simulator_adapter.h"

namespace planning {
namespace simulation {

class SimulatorFactory {
 public:
  static std::unique_ptr<SimulatorAdapter> CreateSimulatorAdapter(
      const std::string& type);
};

}  // namespace simulation
}  // namespace planning

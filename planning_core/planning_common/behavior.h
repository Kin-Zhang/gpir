/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

namespace planning {

enum class LaneChangeStatus {
  kIdle = 0,
  kTryLaneChange = 1,
  kLaneChanging = 2,
  kLaneChangeFailed = 3,
  kLaneChangeCanceled = 4
};

using LCStatus = LaneChangeStatus;

}  // namespace planning

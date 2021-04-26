/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include "planning_core/mock_predictor/mock_predictor.h"

namespace planning {

class ConstVelPredictor : public MockPredictor {
 public:
  ConstVelPredictor(const double prediction_horizon, const double dt)
      : MockPredictor(prediction_horizon, dt) {
    predict_num_ = static_cast<int>(prediction_horizon_ / dt);
  }
  virtual ~ConstVelPredictor() = default;

  bool GeneratePrediction(std::vector<Obstacle>* obstacles) override;

 private:
  int predict_num_ = 0;
};

}  // namespace planning

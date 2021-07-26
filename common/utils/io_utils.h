/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <fstream>
#include <utility>

namespace common {

void DotLog(std::ofstream& os);

template <typename T, typename... Args>
void DotLog(std::ofstream& os, T&& data, Args&&... args) {
  os << std::forward<T>(data) << ",";
  DotLog(os, std::forward<Args>(args)...);
}

}  // namespace common

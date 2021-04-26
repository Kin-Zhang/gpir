/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <vector>
#include <Eigen/Dense>

template<typename T>
using vector_Eigen = std::vector<T, Eigen::aligned_allocator<T>>;
// Copyright 2026 GSoC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AI_CONTROL_BRIDGE_CORE_HPP_
#define AI_CONTROL_BRIDGE_CORE_HPP_

/// @file ai_control_bridge_core.hpp
/// @brief Main header for the AI Control Bridge Core library
/// 
/// Includes the three-stage policy execution pipeline:
/// 1. PreprocessorCore - Prepare observations from robot state
/// 2. InferenceCore - Run neural network policy inference
/// 3. PostprocessorCore - Process actions into robot commands

#include "ai_control_bridge_core/preprocessor_core.hpp"
#include "ai_control_bridge_core/inference_core.hpp"
#include "ai_control_bridge_core/postprocessor_core.hpp"

#endif  // AI_CONTROL_BRIDGE_CORE_HPP_

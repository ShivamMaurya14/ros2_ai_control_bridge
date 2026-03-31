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

#ifndef AI_CONTROL_BRIDGE_CORE__INFERENCE_CORE_HPP_
#define AI_CONTROL_BRIDGE_CORE__INFERENCE_CORE_HPP_

#include <vector>

namespace ai_control_bridge_core
{

/**
 * @brief Abstract base class for inference engines in the policy execution pipeline.
 *
 * Runs neural network policy inference on the preprocessed observation data
 * to produce action commands. Supports any inference backend (ONNX, TensorRT, etc.).
 *
 * This class serves as a plugin base for custom inference implementations.
 */
class InferenceCore
{
public:
  virtual ~InferenceCore() = default;

  /**
   * @brief Run inference on the given observations and produce actions.
   *
   * This pure virtual method must be implemented by derived classes to perform
   * the actual inference computation using the loaded policy model.
   *
   * @param[in] obs The observation vector to use as input for inference.
   * @param[out] action The action vector that will be populated with the inference results.
   * @return true if inference was successful, false otherwise.
   */
  virtual bool run_inference(
    const std::vector<double> & obs, std::vector<double> & action) = 0;
};

}  // namespace ai_control_bridge_core

#endif  // AI_CONTROL_BRIDGE_CORE__INFERENCE_CORE_HPP_

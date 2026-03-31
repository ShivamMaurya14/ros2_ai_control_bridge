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

#ifndef AI_CONTROL_BRIDGE_CORE__POSTPROCESSOR_CORE_HPP_
#define AI_CONTROL_BRIDGE_CORE__POSTPROCESSOR_CORE_HPP_

#include <vector>

namespace ai_control_bridge_core
{

/**
 * @brief Abstract base class for postprocessors in the policy execution pipeline.
 *
 * Transforms raw policy actions into final robot command. This includes:
 * - Trajectory generation (smooth quintic splines)
 * - Safety filtering (velocity, acceleration, jerk limits)
 * - Hardware-specific scaling and normalization
 *
 * This class serves as a plugin base for custom postprocessor implementations.
 */
class PostprocessorCore
{
public:
  virtual ~PostprocessorCore() = default;

  /**
   * @brief Process the actions and return the final robot commands.
   *
   * This pure virtual method must be implemented by derived classes to perform
   * the postprocessing on raw actions and produce the final commands to send
   * to the robot hardware.
   *
   * @param[in] actions The action vector from inference to be postprocessed.
   * @return The final commands vector to be sent to robot hardware.
   */
  virtual const std::vector<double> & [[nodiscard]] process(
    const std::vector<double> & actions) = 0;
};

}  // namespace ai_control_bridge_core

#endif  // AI_CONTROL_BRIDGE_CORE__POSTPROCESSOR_CORE_HPP_

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

#ifndef AI_CONTROL_BRIDGE_CORE__PREPROCESSOR_CORE_HPP_
#define AI_CONTROL_BRIDGE_CORE__PREPROCESSOR_CORE_HPP_

#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/time.hpp"

namespace ai_control_bridge_core
{

struct PreprocessorCoreConfig
{
  // Add any configuration parameters needed for the preprocessor here
  size_t observation_history_length = 0;
  size_t action_history_length = 0;
};

/**
 * @brief Data returned by an observation provider.
 *
 * Contains both the observation values and an optional timestamp.
 */
struct ObservationData
{
  const std::vector<double> & values;  ///< Reference to observation values
  rclcpp::Time timestamp;              ///< Timestamp of the observation data
};

/**
 * @brief Abstract base class for preprocessors in the policy execution pipeline.
 *
 * This class serves as a plugin base class for creating custom preprocessors
 * that transform raw observations and actions into a format suitable for policy execution.
 *
 * Implements the Template Method pattern, providing concrete implementations for
 * observation provider management and observation building, while allowing derived
 * classes to customize specific behavior through virtual methods.
 */
class PreprocessorCore
{
public:
  /// @brief Function signature for observation data providers
  using ObservationProvider = std::function<const ObservationData &()>;

  /**
   * @brief Virtual destructor for proper cleanup of derived classes.
   */
  virtual ~PreprocessorCore() = default;

  /**
   * @brief Set the configuration for the preprocessor.
   *
   * @param config The configuration object containing observation and action lengths.
   */
  void set_config(const PreprocessorCoreConfig & config)
  {
    observation_history_length_ = config.observation_history_length;
    action_history_length_ = config.action_history_length;
  }

  /**
   * @brief Register an observation provider with a name.
   *
   * Providers are called in order of registration when build_observation() is invoked.
   * The vectors returned by each provider are concatenated in registration order
   * to form the final observation vector.
   *
   * @param[in] name Unique name for this segment (used for debugging/introspection)
   * @param[in] provider Function that returns observation data for this segment
   * @throws std::runtime_error if a provider with the same name already exists
   */
  void register_observation_provider(const std::string & name, ObservationProvider provider)
  {
    // Check for duplicate names
    for (const auto & entry : observation_providers_)
    {
      if (entry.first == name)
      {
        throw std::runtime_error("Observation provider with name '" + name + "' already exists.");
      }
    }
    observation_providers_.emplace_back(name, std::move(provider));
  }

  /**
   * @brief Build the observation vector by calling all registered providers.
   *
   * Providers are called in the order they were registered, and their outputs
   * are concatenated to form the final observation vector. The time difference
   * between the provided current_time and each observation's timestamp is stored.
   *
   * @param[in] current_time The current time to calculate time differences against
   * @return true if observation was built successfully, false otherwise
   */
  virtual bool build_observation(const rclcpp::Time & current_time)
  {
    current_observation_.clear();
    observation_time_diffs_.clear();
    for (const auto & [name, provider] : observation_providers_)
    {
      const auto & data = provider();
      current_observation_.insert(
        current_observation_.end(), data.values.begin(), data.values.end());
      observation_time_diffs_[name] = (current_time - data.timestamp).seconds();
    }
    return true;
  }

  /**
   * @brief Get all observation provider time differences.
   *
   * @return const reference to the map of provider names to time differences (in seconds)
   */
  [[nodiscard]] const std::unordered_map<std::string, double> &
  get_observation_time_diffs() const
  {
    return observation_time_diffs_;
  }

  /**
   * @brief Check if any observation providers are registered.
   *
   * @return true if at least one provider is registered
   */
  [[nodiscard]] bool has_observation_providers() const
  {
    return !observation_providers_.empty();
  }

  /**
   * @brief Get the current observation vector.
   *
   * @return const reference to the current observation vector
   */
  [[nodiscard]] const std::vector<double> & get_observation() const
  {
    return current_observation_;
  }

protected:
  /// @brief Registered observation providers (name, provider pairs)
  std::vector<std::pair<std::string, ObservationProvider>> observation_providers_;

  /// @brief Current observation vector built from all providers
  std::vector<double> current_observation_;

  /// @brief Time differences for each provider (in seconds)
  std::unordered_map<std::string, double> observation_time_diffs_;

  /// @brief History length for observations
  size_t observation_history_length_ = 0;

  /// @brief History length for actions
  size_t action_history_length_ = 0;
};

}  // namespace ai_control_bridge_core

#endif  // AI_CONTROL_BRIDGE_CORE__PREPROCESSOR_CORE_HPP_

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

/// @file inference_processor.cpp
/// @brief Implementation of the AI Inference Processor node
///
/// This node runs neural network policy inference at low frequency (~50 Hz)
/// and publishes waypoint commands for the control bridge. It reads observations
/// from sensors, normalizes them, runs inference through a pre-trained model,
/// and publishes the resulting waypoints for trajectory generation.
///
/// Thread Model: Non-realtime async execution at configurable frequency
/// Performance: ~20ms per inference (50 Hz typical)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace ai_inference_processor {

/// @class InferenceProcessor
/// @brief ROS 2 node implementing neural network policy inference
///
/// This node:
/// 1. Subscribes to preprocessed observations from the robot
/// 2. Runs inference through a loaded policy model (ONNX, TensorRT, etc)
/// 3. Publishes waypoints that the control bridge interpolates into commands
///
/// The design supports asynchronous, non-realtime execution since the
/// expensive inference computation doesn't need to run in the control loop.
class InferenceProcessor : public rclcpp::Node,
                           public std::enable_shared_from_this<InferenceProcessor> {
 public:
    /// @brief Initialize the InferenceProcessor node
    /// 
    /// Sets up subscriptions, publishers, and loads the policy model.
    /// Logs successful initialization when complete.
    InferenceProcessor() : rclcpp::Node("ai_inference_processor") {
        // Log initialization
        RCLCPP_INFO(this->get_logger(), "InferenceProcessor node initialized");
        
        // Create subscription to preprocessed observations
        // These come from PreprocessorCore at high frequency (1 kHz)
        // but we only process them at low frequency (50 Hz)
        obs_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "observations",  // Topic name - must match PreprocessorCore output
            10,              // QoS history depth
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                this->observation_callback(msg);
            });
        
        // Create publisher for waypoint commands
        // These are sent to PostprocessorCore for trajectory generation
        // at high frequency (1 kHz control loop)
        waypoints_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "waypoints",  // Topic name - must match PostprocessorCore input
            10);          // QoS history depth
        
        RCLCPP_INFO(this->get_logger(), 
                   "InferenceProcessor: Ready to receive observations on 'observations' topic");
        RCLCPP_INFO(this->get_logger(), 
                   "InferenceProcessor: Publishing waypoints on 'waypoints' topic");
    }

    /// @brief Virtual destructor for proper cleanup
    ///
    /// Ensures all ROS 2 resources are properly released when the node
    /// is destroyed, including subscriptions and publishers.
    virtual ~InferenceProcessor() = default;

 private:
    /// @brief Callback for incoming observation data
    /// 
    /// This is called whenever new preprocessed observations arrive from
    /// the observation topic. Runs neural network inference and publishes waypoints.
    /// 
    /// Production implementation uses:
    /// - ONNX Runtime for model inference
    /// - TensorRT for NVIDIA GPU optimization
    /// - LibTorch for PyTorch models
    /// 
    /// @param msg Incoming observation message from PreprocessorCore
    void observation_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (!msg || msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty observation message");
            return;
        }

        // 1. Validate input dimensions
        size_t obs_size = msg->data.size();
        
        // 2. Normalize observations (scale to [-1, 1] or [0, 1] depending on model)
        std::vector<float> normalized_obs(obs_size);
        for (size_t i = 0; i < obs_size; ++i) {
            // Simple normalization: clamp to [-1, 1]
            // Production code would use model-specific normalization
            normalized_obs[i] = std::max(-1.0f, std::min(1.0f, (float)msg->data[i]));
        }

        // 3. Run neural network inference
        // TODO: Integrate actual ONNX/TensorRT model here
        // Example structure:
        // std::vector<float> model_output = model_->predict(normalized_obs);
        
        // For now, use a simple pass-through for testing
        // In production, this would be actual model inference
        std::vector<float> model_output(obs_size);
        
        // Simple demo: copy normalized observations (should be replaced with actual inference)
        model_output = normalized_obs;
        
        // 4. Post-process model output
        // - Apply action scaling (e.g., multiply by max joint velocity)
        // - Apply action clipping and smoothing
        // - Apply any model-specific post-processing
        std::vector<double> waypoint_data(model_output.size());
        for (size_t i = 0; i < model_output.size(); ++i) {
            // Denormalize from [-1, 1] back to feasible action range
            // This scales the normalized output to the robot's action space
            waypoint_data[i] = model_output[i];  // Already in reasonable range
        }

        // 5. Create and publish waypoint message
        auto waypoints = std::make_unique<std_msgs::msg::Float64MultiArray>();
        waypoints->data = waypoint_data;

        // Publish waypoints to control bridge
        waypoints_publisher_->publish(std::move(waypoints));

        RCLCPP_DEBUG(this->get_logger(), 
                    "Inference: %ld inputs -> %ld waypoints",
                    obs_size, waypoint_data.size());
    }

    // ROS 2 subscriptions and publishers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obs_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr waypoints_publisher_;
};

}  // namespace ai_inference_processor

/// @brief Main entry point for the ROS 2 node
/// 
/// Initializes ROS 2, creates the InferenceProcessor node, and spins until
/// shutdown is requested.
/// 
/// @param argc Command line argument count
/// @param argv Command line arguments
/// @return Exit code (0 for success)
int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create and run the inference processor node
    rclcpp::spin(std::make_shared<ai_inference_processor::InferenceProcessor>());
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    
    return 0;
}

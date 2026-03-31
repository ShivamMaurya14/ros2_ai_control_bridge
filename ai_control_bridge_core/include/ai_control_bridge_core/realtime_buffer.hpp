#pragma once

#include <vector>
#include <cstdint>

namespace ai_control_bridge_core {

/**
 * @brief Waypoint structure for AI policy output.
 * 
 * Represents target joint positions from AI policy inference.
 * Can include velocity information for feed-forward control.
 */
struct Waypoint {
    std::vector<double> position;    ///< Target joint positions
    std::vector<double> velocity;    ///< Target joint velocities (optional)
    uint64_t timestamp_ns;           ///< Timestamp in nanoseconds
    uint32_t sequence;               ///< Sequence number for ordering

    /// Constructor initializing with given number of joints
    explicit Waypoint(size_t num_joints = 6)
        : position(num_joints, 0.0),
          velocity(num_joints, 0.0),
          timestamp_ns(0),
          sequence(0) {}
};

/**
 * @brief Abstract interface for realtime-safe buffering.
 * 
 * Provides lock-free communication between non-realtime inference thread
 * and realtime control loop. Uses atomic operations only.
 * 
 * Thread Model:
 * - write() called from non-realtime thread (inference)
 * - read/peek called from realtime thread (control loop)
 */
class RealtimeBuffer {
public:
    virtual ~RealtimeBuffer() = default;

    /**
     * @brief Write waypoint (non-realtime thread safe).
     * 
     * Called from non-realtime inference thread to publish new waypoint.
     * Does not block or use locks.
     * 
     * @param waypoint Waypoint to write
     */
    virtual void write(const Waypoint& waypoint) = 0;

    /**
     * @brief Read latest available waypoint (realtime safe).
     * 
     * Called from realtime control thread.
     * Lock-free and wait-free.
     * Only returns new data if it has not been read before.
     * 
     * @param waypoint[out] Waypoint to populate
     * @return true if new waypoint was available, false if no new data
     */
    virtual bool read(Waypoint& waypoint) = 0;

    /**
     * @brief Peek at latest waypoint without consuming.
     * 
     * Called from realtime control thread.
     * Returns the most recent waypoint without marking it as read.
     * Allows inspecting data before committing to it.
     * 
     * @param waypoint[out] Waypoint to populate
     * @return true if waypoint available, false otherwise
     */
    virtual bool peek(Waypoint& waypoint) const = 0;

    /**
     * @brief Check if new data is available without reading.
     * 
     * @return true if unread waypoints are available
     */
    virtual bool has_data() const = 0;

    /**
     * @brief Get number of buffered waypoints available to read.
     * 
     * @return Number of waypoints in buffer waiting to be read
     */
    virtual size_t buffered_count() const = 0;

    /**
     * @brief Clear all buffered waypoints.
     * 
     * @param sequence Starting sequence number for next waypoint
     */
    virtual void clear(uint32_t sequence = 0) = 0;

    /**
     * @brief Get capacity of the buffer.
     * 
     * @return Maximum number of waypoints that can be buffered
     */
    virtual size_t capacity() const = 0;
};

}  // namespace ai_control_bridge_core

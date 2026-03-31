#pragma once

#include "../realtime_buffer.hpp"
#include <array>
#include <atomic>
#include <algorithm>

namespace ai_control_bridge_core::impl {

/**
 * @brief Realtime-safe ring buffer for waypoint data.
 * 
 * Provides lock-free communication between non-realtime inference thread
 * and realtime control loop using double-buffering pattern.
 * 
 * Features:
 * - No dynamic allocation after construction
 * - No locks in write/read paths  
 * - Atomic operations only
 * - Supports any number of joints
 */
template <size_t CAPACITY = 200>
class RealtimeSafeBuffer : public RealtimeBuffer {
public:
    explicit RealtimeSafeBuffer(size_t num_joints = 6)
        : write_idx_(0), read_idx_(0), sequence_(0), num_joints_(num_joints) {
        // Pre-allocate buffer with properly-sized waypoints
        for (size_t i = 0; i < CAPACITY; ++i) {
            buffer_[i] = Waypoint(num_joints);
        }
    }

    virtual ~RealtimeSafeBuffer() = default;

    /// Write waypoint from non-realtime thread
    /// 
    /// Memory ordering: Uses relaxed loads/stores for performance since
    /// the writer doesn't need to synchronize with readers until the
    /// index update. The final store uses memory_order_release to ensure
    /// the written data is visible before the index update.
    ///
    /// @param waypoint The waypoint data to write
    virtual void write(const Waypoint& waypoint) override {
        uint32_t wi = write_idx_.load(std::memory_order_relaxed);
        uint32_t next = (wi + 1) % CAPACITY;
        buffer_[wi] = waypoint;  // Write with correct index loaded once
        write_idx_.store(next, std::memory_order_release);
    }

    /// Read latest waypoint from realtime thread (consumes data)
    virtual bool read(Waypoint& waypoint) override {
        uint32_t wi = write_idx_.load(std::memory_order_acquire);
        uint32_t ri = read_idx_.load(std::memory_order_relaxed);
        
        if (wi == ri) {
            return false;  // No new data
        }
        
        waypoint = buffer_[ri];
        read_idx_.store((ri + 1) % CAPACITY, std::memory_order_release);
        return true;
    }

    /// Peek latest waypoint from realtime thread (non-consuming)
    virtual bool peek(Waypoint& waypoint) const override {
        uint32_t wi = write_idx_.load(std::memory_order_acquire);
        uint32_t ri = read_idx_.load(std::memory_order_relaxed);
        
        if (wi == ri) return false;  // No new data since last read
        
        // Get the most recent data (which is at wi-1)
        uint32_t idx = (wi - 1 + CAPACITY) % CAPACITY;
        waypoint = buffer_[idx];
        return true;
    }

    virtual bool has_data() const override {
        return write_idx_.load(std::memory_order_acquire) != 
               read_idx_.load(std::memory_order_relaxed);
    }

    virtual size_t buffered_count() const override {
        uint32_t wi = write_idx_.load(std::memory_order_acquire);
        uint32_t ri = read_idx_.load(std::memory_order_relaxed);
        return (wi - ri + CAPACITY) % CAPACITY;
    }

    virtual void clear(uint32_t sequence = 0) override {
        read_idx_.store(write_idx_.load(std::memory_order_acquire), 
                       std::memory_order_release);
        sequence_ = sequence;
    }

    virtual size_t capacity() const override { return CAPACITY; }

private:
    std::array<Waypoint, CAPACITY> buffer_;
    std::atomic<uint32_t> write_idx_;
    std::atomic<uint32_t> read_idx_;
    uint32_t sequence_;
    size_t num_joints_;
};

}  // namespace ai_control_bridge_core::impl

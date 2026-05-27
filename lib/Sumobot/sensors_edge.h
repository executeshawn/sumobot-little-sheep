// ============================================================================
//  sensors_edge.h - QTR-MD-03RC reflectance array (white ring edge detect)
// ============================================================================
#pragma once
#include <stdint.h>

// Which side(s) of the array see the white border.
enum class EdgeDir : uint8_t {
    NONE,
    LEFT,
    CENTER,
    RIGHT,
    FRONT // multiple/whole front lit
};

namespace Edge {

void begin();

// Spin-in-place calibration sweep helper: call repeatedly while rotating so
// every sensor sees both black mat and white border. Internally accumulates
// min/max per sensor. Call many times across one slow rotation.
void calibrateEdge();

// FAST loop: perform one RC read and cache the result. This is the only place
// the array is actually read at runtime; the queries below are non-blocking
// and just return the cached values, so the FSM can check the edge first on
// every tick without paying I/O latency.
void poll();

// True if any sensor saw the white ring at the last poll().
bool isOnEdge();

// Which sensor(s) tripped at the last poll(), for choosing the retreat side.
EdgeDir edgeDirection();

} // namespace Edge

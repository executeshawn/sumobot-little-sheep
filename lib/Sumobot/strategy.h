// ============================================================================
//  strategy.h - top-level finite state machine
// ============================================================================
#pragma once
#include <stdint.h>

enum class State : uint8_t {
    IDLE,
    CALIBRATE,
    COUNTDOWN,
    SEARCH,
    ATTACK,
    EVADE,
    STOP
};

namespace Strategy {

void begin();

// Latches a critical fault; the FSM moves to STOP on the next update().
void setFault(const char *reason);

// Call once per loop(). Reads inputs, runs the active state, drives outputs.
void update();

State currentState();
const char *stateName(State s);

} // namespace Strategy

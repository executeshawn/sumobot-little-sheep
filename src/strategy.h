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

// Decoded DIP switch behaviour flags (true = switch ON = pin LOW).
// Reduced to 3 switches in the current hardware revision.
struct DipSettings {
    bool aggressive;  // SW1: higher attack speed, shorter detection range
    bool spinSearch;  // SW2: spin-in-place search instead of sweep
    bool torqueMode;  // SW3: cap PWM for holding force
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

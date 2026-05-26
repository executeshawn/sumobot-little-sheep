// ============================================================================
//  TEST: FSM transition simulator (no hardware required)
// ============================================================================
//
//  Purpose
//    Validate that every transition in the supervisory FSM described in
//    src/strategy.cpp is reachable and correctly guarded. This test does
//    NOT exercise real sensors or motors; instead it mirrors the state
//    table in a self-contained simulator driven by single-character Serial
//    commands ("mocked sensor triggers").
//
//    Time-based transitions (CALIBRATE -> COUNTDOWN, COUNTDOWN -> SEARCH,
//    and the ATTACK 10 s watchdog) fire automatically using the same
//    durations as the real firmware.
//
//  Why a separate simulator instead of running strategy.cpp directly?
//    The real Strategy::update() consumes Edge::isOnEdge() / ToF::detect-
//    Opponent() / digitalRead() at every tick. Replacing those at the
//    test level would require build-flag injection of mocks into the
//    shared src/ tree, which is out of scope here. Mirroring the table
//    keeps the test 100 % isolated and makes it the canonical, runnable
//    cross-check that the documented state diagram is internally
//    consistent.
//
//  Commands (single character via Serial monitor):
//      b   start button press         (IDLE -> CALIBRATE)
//      o   opponent detected          (SEARCH -> ATTACK)
//      n   opponent lost              (ATTACK -> SEARCH)
//      e   edge detected              (SEARCH/ATTACK -> EVADE)
//      s   edge clear                 (EVADE -> SEARCH)
//      f   fault                      (any -> STOP)
//      r   reset                      (any -> IDLE)
//      ?   print this help
//
//  Flash:  pio test -e esp32s3 -f test_fsm
//  Watch:  pio device monitor -b 115200
//
#include <Arduino.h>

enum class TS : uint8_t { IDLE, CALIBRATE, COUNTDOWN, SEARCH, ATTACK, EVADE, STOP };

static const char *tsName(TS s) {
    switch (s) {
        case TS::IDLE:      return "IDLE";
        case TS::CALIBRATE: return "CALIBRATE";
        case TS::COUNTDOWN: return "COUNTDOWN";
        case TS::SEARCH:    return "SEARCH";
        case TS::ATTACK:    return "ATTACK";
        case TS::EVADE:     return "EVADE";
        case TS::STOP:      return "STOP";
    }
    return "?";
}

// Match the durations from src/config.h / src/strategy.cpp.
constexpr uint32_t CAL_MS    = 4000;
constexpr uint32_t COUNT_MS  = 5000;
constexpr uint32_t ATK_WD_MS = 10000;

static TS       g_state        = TS::IDLE;
static uint32_t g_stateEntered = 0;
static uint32_t g_attackStart  = 0;

static void enter(TS next, const char *cause) {
    Serial.printf("[FSM] %-9s -> %-9s  (cause: %s, t=%lu ms)\n",
                  tsName(g_state), tsName(next), cause, (unsigned long)millis());
    g_state = next;
    g_stateEntered = millis();
    if (next == TS::ATTACK) g_attackStart = millis();
}

static void printHelp() {
    Serial.println("\nCommands: b=button  o=opponent  n=opponent-lost  "
                   "e=edge  s=edge-clear  f=fault  r=reset  ?=help\n");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== TEST: FSM transition simulator ===");
    printHelp();
    Serial.printf("[FSM] initial state = %s\n", tsName(g_state));
}

void loop() {
    // -- Time-based transitions (mirror the real FSM exactly) ----------
    const uint32_t inState = millis() - g_stateEntered;

    if (g_state == TS::CALIBRATE && inState >= CAL_MS) {
        enter(TS::COUNTDOWN, "calibration timer");
    } else if (g_state == TS::COUNTDOWN && inState >= COUNT_MS) {
        enter(TS::SEARCH, "5 s countdown elapsed");
    } else if (g_state == TS::ATTACK &&
               (millis() - g_attackStart) >= ATK_WD_MS) {
        enter(TS::SEARCH, "ATTACK watchdog 10 s");
    }

    // -- Event-driven transitions (from Serial mock commands) ----------
    if (Serial.available()) {
        const char c = Serial.read();
        if (c == '\r' || c == '\n' || c == ' ') return;

        switch (c) {
        case 'b':
            if (g_state == TS::IDLE) enter(TS::CALIBRATE, "button press");
            else Serial.println("  (button ignored - only valid in IDLE)");
            break;
        case 'o':
            if (g_state == TS::SEARCH) enter(TS::ATTACK, "opponent detected");
            else Serial.println("  (opponent ignored - only valid in SEARCH)");
            break;
        case 'n':
            if (g_state == TS::ATTACK) enter(TS::SEARCH, "opponent lost");
            else Serial.println("  (n ignored - only valid in ATTACK)");
            break;
        case 'e':
            if (g_state == TS::SEARCH || g_state == TS::ATTACK)
                enter(TS::EVADE, "edge detected (priority override)");
            else Serial.println("  (edge ignored - only valid in SEARCH/ATTACK)");
            break;
        case 's':
            if (g_state == TS::EVADE) enter(TS::SEARCH, "edge clear");
            else Serial.println("  (s ignored - only valid in EVADE)");
            break;
        case 'f':
            enter(TS::STOP, "fault");
            break;
        case 'r':
            enter(TS::IDLE, "reset");
            break;
        case '?':
            printHelp();
            break;
        default:
            Serial.printf("  unknown command '%c' - send '?' for help\n", c);
            break;
        }
    }

    delay(10);
}

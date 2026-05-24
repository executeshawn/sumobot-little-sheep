// ============================================================================
//  strategy.cpp - sumo behaviour finite state machine
// ============================================================================
#include "strategy.h"
#include "pins.h"
#include "config.h"
#include "motors.h"
#include "sensors_tof.h"
#include "sensors_edge.h"
#include <Arduino.h>

namespace {

constexpr uint32_t CALIBRATE_DURATION_MS = 4000; // slow-spin while QTR learns
constexpr uint32_t COUNTDOWN_BLINK_MS    = 500;  // 1 blink/sec (on/off 500ms)

State    g_state        = State::IDLE;
uint32_t g_stateEntered = 0;
uint32_t g_attackStart  = 0;   // for the 10 s attack watchdog
bool     g_fault        = false;
const char *g_faultMsg  = "";

// EVADE sub-phases
enum class EvadePhase : uint8_t { REVERSE, TURN };
EvadePhase g_evadePhase = EvadePhase::REVERSE;
uint32_t   g_evadePhaseStart = 0;
bool       g_evadeTurnRight  = true;
bool       g_evadeDirChosen  = false;

// SEARCH sweep phase
uint32_t g_sweepStart = 0;
bool     g_sweepLeft  = true;

// Start-button debounce
bool     g_btnPrev = false; // tracks last raw "pressed" level (idle = false)

// ---- helpers ------------------------------------------------------------

void enter(State s) {
    g_state = s;
    g_stateEntered = millis();
    if (s == State::ATTACK) g_attackStart = millis();
    if (s == State::EVADE) {
        g_evadePhase = EvadePhase::REVERSE;
        g_evadePhaseStart = millis();
        g_evadeDirChosen = false;
    }
    if (s == State::SEARCH) {
        g_sweepStart = millis();
    }
}

uint32_t inState() { return millis() - g_stateEntered; }

void ledOn()  { digitalWrite(PIN_STATUS_LED, HIGH); }
void ledOff() { digitalWrite(PIN_STATUS_LED, LOW); }
void ledBlink(uint32_t halfPeriodMs) {
    digitalWrite(PIN_STATUS_LED, ((millis() / halfPeriodMs) & 1) ? HIGH : LOW);
}

DipSettings readDip() {
    DipSettings d;
    d.aggressive  = (digitalRead(PIN_SW1) == LOW);
    d.spinSearch  = (digitalRead(PIN_SW2) == LOW);
    d.torqueMode  = (digitalRead(PIN_SW3) == LOW);
    return d;
}

bool startPressed() {
    bool now = (digitalRead(PIN_START_BTN) == LOW);
    bool edge = now && g_btnPrev == false; // falling edge (HIGH->LOW)
    // store as "was HIGH" inverted: track previous raw level
    g_btnPrev = now;
    return edge;
}

// Steer toward an opponent bearing at the given attack speed.
void driveTowards(OpponentDir dir, int spd) {
    const int slow = (spd * 3) / 5; // inner-wheel speed when veering
    switch (dir) {
        case OpponentDir::FRONT_CENTER: Motors::forward(spd);            break;
        case OpponentDir::FRONT_LEFT:   Motors::drive(slow, spd);        break;
        case OpponentDir::FRONT_RIGHT:  Motors::drive(spd, slow);        break;
        case OpponentDir::LEFT:         Motors::turnLeft(SPEED_TURN);    break;
        case OpponentDir::RIGHT:        Motors::turnRight(SPEED_TURN);   break;
        case OpponentDir::REAR:         Motors::turnRight(SPEED_TURN);   break; // spin to face
        default:                        Motors::stop();                  break;
    }
}

} // namespace

namespace Strategy {

void begin() {
    pinMode(PIN_SW1, INPUT_PULLUP);
    pinMode(PIN_SW2, INPUT_PULLUP);
    pinMode(PIN_SW3, INPUT_PULLUP);
    pinMode(PIN_START_BTN, INPUT_PULLUP);
    pinMode(PIN_STATUS_LED, OUTPUT);
    ledOff();
    enter(State::IDLE);
}

void setFault(const char *reason) {
    g_fault = true;
    g_faultMsg = reason;
}

void update() {
    const DipSettings dip = readDip();

    // Apply speed cap from torque mode every cycle.
    Motors::setSpeedCap(dip.torqueMode ? SPEED_TORQUE_CAP : SPEED_FULL);

    // Critical fault overrides everything.
    if (g_fault && g_state != State::STOP) {
        Serial.printf("[FSM] FAULT -> STOP: %s\n", g_faultMsg);
        enter(State::STOP);
    }

    switch (g_state) {

    case State::IDLE:
        Motors::stop();
        ledBlink(LED_BLINK_SLOW);
        if (startPressed()) {
            Serial.println("[FSM] IDLE -> CALIBRATE (start pressed)");
            enter(State::CALIBRATE);
        }
        break;

    case State::CALIBRATE:
        ledBlink(LED_BLINK_FAST);
        Motors::turnRight(SPEED_SEARCH); // slow spin so the QTR sees mat+border
        Edge::calibrateEdge();
        if (inState() >= CALIBRATE_DURATION_MS) {
            Motors::stop();
            Serial.println("[FSM] CALIBRATE -> COUNTDOWN");
            enter(State::COUNTDOWN);
        }
        break;

    case State::COUNTDOWN:
        Motors::stop();
        ledBlink(COUNTDOWN_BLINK_MS); // 1 blink/sec
        if (inState() >= START_DELAY_MS) {
            Serial.println("[FSM] COUNTDOWN -> SEARCH");
            enter(State::SEARCH);
        }
        break;

    case State::SEARCH: {
        ledOn();
        if (Edge::isOnEdge()) { enter(State::EVADE); break; }

        TofReadings r = ToF::readAllSensors();
        OpponentDir opp = ToF::detectOpponent(r, dip.aggressive);
        if (opp != OpponentDir::NONE) {
            Serial.printf("[FSM] SEARCH -> ATTACK (%s)\n", ToF::dirName(opp));
            enter(State::ATTACK);
            break;
        }

        if (dip.spinSearch) {
            Motors::turnRight(SPEED_SEARCH); // continuous spin
        } else {
            // sweep: alternate pivot direction each segment
            if (millis() - g_sweepStart >= SWEEP_SEGMENT_MS) {
                g_sweepLeft = !g_sweepLeft;
                g_sweepStart = millis();
            }
            if (g_sweepLeft) Motors::turnLeft(SPEED_SEARCH);
            else             Motors::turnRight(SPEED_SEARCH);
        }
        break;
    }

    case State::ATTACK: {
        ledBlink(LED_BLINK_RAPID);
        if (Edge::isOnEdge()) { enter(State::EVADE); break; }

        // Watchdog: stuck attacking too long -> rescan.
        if (millis() - g_attackStart >= ATTACK_WATCHDOG_MS) {
            Serial.println("[FSM] ATTACK watchdog -> SEARCH");
            enter(State::SEARCH);
            break;
        }

        TofReadings r = ToF::readAllSensors();
        OpponentDir opp = ToF::detectOpponent(r, dip.aggressive);
        if (opp == OpponentDir::NONE) {
            enter(State::SEARCH); // lost the target
            break;
        }
        const int atkSpeed = dip.aggressive ? SPEED_ATTACK_AGGRESSIVE
                                             : SPEED_ATTACK;
        driveTowards(opp, atkSpeed);
        break;
    }

    case State::EVADE:
        ledOn(); // EVADE indicator (single LED held solid)
        if (g_evadePhase == EvadePhase::REVERSE) {
            // Decide turn direction from which side saw the edge (once).
            if (!g_evadeDirChosen) {
                EdgeDir e = Edge::edgeDirection();
                g_evadeTurnRight = (e != EdgeDir::RIGHT); // turn away from edge
                g_evadeDirChosen = true;
            }
            Motors::reverse(SPEED_TURN);
            if (millis() - g_evadePhaseStart >= EVADE_REVERSE_MS) {
                g_evadePhase = EvadePhase::TURN;
                g_evadePhaseStart = millis();
            }
        } else { // TURN ~180 deg
            if (g_evadeTurnRight) Motors::turnRight(SPEED_TURN);
            else                  Motors::turnLeft(SPEED_TURN);
            if (millis() - g_evadePhaseStart >= EVADE_TURN_MS) {
                if (!Edge::isOnEdge()) {
                    Serial.println("[FSM] EVADE -> SEARCH (clear)");
                    enter(State::SEARCH);
                } else {
                    // still on edge: reverse again
                    g_evadePhase = EvadePhase::REVERSE;
                    g_evadePhaseStart = millis();
                }
            }
        }
        break;

    case State::STOP:
        Motors::stop();        // also pulls STBY LOW
        digitalWrite(PIN_STBY, LOW);
        ledOff();
        break;
    }
}

State currentState() { return g_state; }

const char *stateName(State s) {
    switch (s) {
        case State::IDLE:      return "IDLE";
        case State::CALIBRATE: return "CALIBRATE";
        case State::COUNTDOWN: return "COUNTDOWN";
        case State::SEARCH:    return "SEARCH";
        case State::ATTACK:    return "ATTACK";
        case State::EVADE:     return "EVADE";
        case State::STOP:      return "STOP";
        default:               return "?";
    }
}

} // namespace Strategy

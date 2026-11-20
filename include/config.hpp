#pragma once

// Every port, measurement and tuning number for the rebuild, in one place.
//
// "PLANNED" = the design target we picked. Build to it, then measure the real robot and fix
// the number if it came out different.
// "MEASURE" = can only be read off the built robot. Fill it in; don't guess.

#include <cstdint>
#include "lemlib/chassis/trackingWheel.hpp"

namespace cfg {

// ============================================================================
// Ports (all 21 used: 8 motors + radio + 12 sensors).
// Change these to match how the robot is wired. A minus sign reverses a motor or sensor.
// ============================================================================

// Drive: 4 x 11 W, 450 rpm, 3.25" omnis
constexpr std::int8_t LEFT_FRONT = -1;
constexpr std::int8_t LEFT_BACK = -2;
constexpr std::int8_t RIGHT_FRONT = 3;
constexpr std::int8_t RIGHT_BACK = 4;

// Cascade: 3 x 11 W (33 W); its spool also swings the toggle arm. Intake: its own 11 W motor.
constexpr std::int8_t CASCADE_A = 5;
constexpr std::int8_t CASCADE_B = -6;
constexpr std::int8_t CASCADE_C = 8;
constexpr std::int8_t INTAKE_MOTOR = 7;

constexpr std::uint8_t RIGHT_OPTICAL = 9;  // toggle optical, right side

// Sensors
constexpr std::uint8_t IMU_A = 10;
constexpr std::uint8_t IMU_B = 11;
constexpr std::int8_t TRACK_VERTICAL = 12;   // rotation sensor, wheel rolls forward/back
constexpr std::int8_t TRACK_HORIZONTAL = 13; // rotation sensor, wheel rolls left/right
constexpr std::uint8_t DIST_LEFT = 14;       // wall distance sensors
constexpr std::uint8_t DIST_RIGHT = 15;
constexpr std::uint8_t DIST_FRONT = 16;
constexpr std::uint8_t CLAW_DIST = 17;
constexpr std::uint8_t LEFT_OPTICAL = 18;  // toggle optical, left side
constexpr std::int8_t CASCADE_ROT = 19;
constexpr std::uint8_t FRONT_OPTICAL = 20; // toggle optical, front
// port 21: radio

// Pistons (3-wire ports on the brain)
constexpr char CLAW_PISTON = 'A';     // single-acting, rubber bands return it
constexpr char ARM_FLIP_PISTON = 'B'; // double-acting
constexpr char BALANCER_PISTON = 'C';  // balancers: come out by themselves when the cascade is at the top

// ============================================================================
// Drive + odometry
// ============================================================================

// 6 x 3.25" omnis (3 per side), all omni, 450 rpm: about 77 in/s top speed
constexpr float TRACK_WIDTH = 12;      // PLANNED (lowest stable with the cascade + intake): middle of left wheels to middle of right wheels, in
constexpr float DRIVE_RPM = 450;
constexpr float HORIZONTAL_DRIFT = 2;  // LemLib: 2 for an all-omni drive

constexpr float TRACK_WHEEL_DIAMETER = lemlib::Omniwheel::NEW_2; // 2" omnis (2.125 real size), spring-loaded down
// Offsets from the robot's turning center, inches.
// Vertical wheel: + = right of center. Horizontal wheel: + = in front of center.
// Both wheels go as close to the turning center as they fit (less error when turning).
constexpr float VERTICAL_OFFSET = -0.25; // PLANNED: cross-brace hole 15, 0.25" left of the center line
constexpr float HORIZONTAL_OFFSET = -1;  // PLANNED: drive-rail hole 20, 1" behind the turning center (hole 18)

// If one IMU always reads a little more or less than the other over 10 turns, fix it here
// (1.0 = no change). Measure: spin 10 full turns by hand, scale = 3600 / reading.
constexpr double IMU_A_SCALE = 1.0;
constexpr double IMU_B_SCALE = 1.0;

// ============================================================================
// Wall distance sensors (for position resets)
// Where each sensor sits, from the turning center: x = + right, y = + forward, inches.
// ============================================================================

// Side sensors level with the turning center, back sensor on the center line.
constexpr float DIST_LEFT_X = -7.5, DIST_LEFT_Y = 0;   // PLANNED
constexpr float DIST_RIGHT_X = 7.5, DIST_RIGHT_Y = 0;  // PLANNED
constexpr float DIST_FRONT_X = 0, DIST_FRONT_Y = 8.5;  // PLANNED
constexpr float WALL_RESET_MAX_ANGLE = 8; // only reset if the sensor points within this many degrees of square to the wall
constexpr float WALL_RESET_MAX_JUMP = 8;  // ignore a reading that moves the robot more than this (it saw a goal, not the wall)
constexpr int WALL_RESET_MAX_MM = 2000;   // past this the sensor isn't reliable

// ============================================================================
// Cascade (3 stages, 17.5" each, 2.5" overlap: 30" of travel)
// ============================================================================

constexpr double CASCADE_TRAVEL = 30;        // inches from all the way down to all the way up
constexpr double CASCADE_IN_PER_DEG = 0;     // MEASURE: inches the claw rises per cascade rotation-sensor degree
                                             // (raise to the top, CASCADE_TRAVEL / degrees shown on the screen)
constexpr double CLAW_HEIGHT_DOWN = 0;       // MEASURE: floor to the bottom of the closed claw, cascade all the way down, in
constexpr double CASCADE_KP = 25;            // power per inch of error
constexpr int CASCADE_MIN_POWER = 20;        // enough to move it at all
constexpr double CASCADE_TOLERANCE = 0.3;    // inches: close enough, hold
constexpr int CASCADE_HOLD_POWER = 8;        // gentle push up while holding (gravity)
// Balancers: out when the claw rise reaches BALANCER_OUT_AT, back in below BALANCER_IN_AT
// (the gap stops them flicking in and out at the top).
constexpr double BALANCER_OUT_AT = CASCADE_TRAVEL - 1;
constexpr double BALANCER_IN_AT = CASCADE_TRAVEL - 3;
constexpr bool BALANCER_OUT_IS_EXTENDED = true; // PLANNED: air pushes them out

// ============================================================================
// Claw
// ============================================================================

constexpr bool CLAW_OUT_IS_CLOSED = true;    // PLANNED: air closes the claw (strong grip), rubber bands open it
constexpr int CLAW_MS = 200;                 // time to close / open
constexpr int CLAW_PIN_MM = 0;               // MEASURE: claw distance sensor reading with a pin in the claw (+ a few mm)

// Match loading. Main way: dropped into the claw from the back, then close and lift.
constexpr int LOAD_SETTLE_MS = 100;          // after the sensor sees the load: time for it to drop all the way in
constexpr int LOAD_TIMEOUT_MS = 3000;        // give up waiting for a load after this
constexpr double LOAD_LIFT = 2;              // inches to lift right after closing (clears the loader)

// ============================================================================
// Toggle optical: which color the toggle shows. Starting values: check them on a real toggle
// (the check screen shows the hue) and fix if needed.
// ============================================================================

constexpr double RED_HUE_MAX = 30;           // red: hue below this ...
constexpr double RED_HUE_MIN = 330;          // ... or above this
constexpr double BLUE_HUE_MIN = 190, BLUE_HUE_MAX = 250;
constexpr int TOGGLE_MIN_PROXIMITY = 100;    // 0-255: closer than this counts as "sees a toggle"

// ============================================================================
// Toggle arm: no motor of its own. The cascade spool swings it, so raising the cascade runs it.
// ============================================================================

constexpr double TOGGLE_HIT_IN = 0;          // MEASURE: cascade rise (in) where the toggle arm is all the way out

// ============================================================================
// Pre-auto check
// ============================================================================

constexpr int BATTERY_MIN_PERCENT = 80;
constexpr double IMU_MAX_DISAGREE = 2;       // degrees between the two IMUs before the check turns red

} // namespace cfg

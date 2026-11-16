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
// Ports (all 21 used: 9 motors + radio + 11 sensors).
// Change these to match how the robot is wired. A minus sign reverses a motor or sensor.
// ============================================================================

// Drive: 4 x 11 W, 450 rpm, 2.75" omnis
constexpr std::int8_t LEFT_FRONT = -1;
constexpr std::int8_t LEFT_BACK = -2;
constexpr std::int8_t RIGHT_FRONT = 3;
constexpr std::int8_t RIGHT_BACK = 4;

// Cascade + intake: 2 motors always on the cascade, the 3rd one shifts with the PTO
constexpr std::int8_t CASCADE_A = 5;
constexpr std::int8_t CASCADE_B = -6;
constexpr std::int8_t PTO_MOTOR = 7;

// Toggle arms: 2 x 5.5 W
constexpr std::int8_t ARM_LEFT = 8;
constexpr std::int8_t ARM_RIGHT = -9;

// Sensors
constexpr std::uint8_t IMU_A = 10;
constexpr std::uint8_t IMU_B = 11;
constexpr std::int8_t TRACK_VERTICAL = 12;   // rotation sensor, wheel rolls forward/back
constexpr std::int8_t TRACK_HORIZONTAL = 13; // rotation sensor, wheel rolls left/right
constexpr std::uint8_t DIST_LEFT = 14;       // wall distance sensors
constexpr std::uint8_t DIST_RIGHT = 15;
constexpr std::uint8_t DIST_BACK = 16;
constexpr std::uint8_t CLAW_DIST = 17;
constexpr std::int8_t ARM_ROT = 18;
constexpr std::int8_t CASCADE_ROT = 19;
constexpr std::uint8_t CLAW_OPTICAL = 20;
// port 21: radio

// Pistons (3-wire ports on the brain)
constexpr char CLAW_PISTON = 'A';     // single-acting, rubber bands return it
constexpr char PTO_PISTON = 'B';      // single-acting, rubber bands return it
constexpr char ARM_FLIP_PISTON = 'C'; // double-acting

// ============================================================================
// Drive + odometry
// ============================================================================

// 6 x 2.75" omnis (3 per side), all omni, 450 rpm: about 65 in/s top speed
constexpr float TRACK_WIDTH = 12.5;    // PLANNED: middle of left wheels to middle of right wheels, in
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
constexpr float DIST_BACK_X = 0, DIST_BACK_Y = -8.5;   // PLANNED
constexpr float WALL_RESET_MAX_ANGLE = 8; // only reset if the sensor points within this many degrees of square to the wall
constexpr float WALL_RESET_MAX_JUMP = 8;  // ignore a reading that moves the robot more than this (it saw a goal, not the wall)
constexpr int WALL_RESET_MAX_MM = 2000;   // past this the sensor isn't reliable

// ============================================================================
// Cascade (3 stages, 17.5" each, 2.5" overlap: 30" of travel)
// ============================================================================

constexpr double CASCADE_TRAVEL = 30;        // inches from all the way down to all the way up
constexpr double CASCADE_IN_PER_DEG = 0;     // MEASURE: inches the claw rises per rotation-sensor degree
                                             // (raise to the top, CASCADE_TRAVEL / degrees shown on the screen)
constexpr double CLAW_HEIGHT_DOWN = 0;       // MEASURE: floor to the bottom of the closed claw, cascade all the way down, in
constexpr double CASCADE_KP = 25;            // power per inch of error
constexpr int CASCADE_MIN_POWER = 20;        // enough to move it at all
constexpr double CASCADE_TOLERANCE = 0.3;    // inches: close enough, hold
constexpr int CASCADE_HOLD_POWER = 8;        // gentle push up while holding (gravity)

// Which way the PTO piston shifts the 3rd motor. PLANNED: piston out = intake, rubber bands
// pull it back to the cascade. So if the air runs out, the lift still has all 3 motors.
constexpr bool PTO_OUT_IS_INTAKE = true;
constexpr int PTO_SHIFT_MS = 150;            // time for the gears to slide over

// ============================================================================
// Claw
// ============================================================================

constexpr bool CLAW_OUT_IS_CLOSED = true;    // PLANNED: air closes the claw (strong grip), rubber bands open it
constexpr int CLAW_MS = 200;                 // time to close / open
constexpr int CLAW_PIN_MM = 0;               // MEASURE: claw distance sensor reading with a pin in the claw (+ a few mm)

// ============================================================================
// Toggle arms (rotation sensor on the arm)
// ============================================================================

constexpr double ARM_REST_DEG = 0;           // MEASURE: arm sensor reading with the arm at rest (start position)
constexpr double ARM_OUT_DEG = 0;            // MEASURE: reading with the arm out to hit a toggle
constexpr double ARM_ZERO_TOLERANCE = 5;     // pre-auto check: arm counts as zeroed within this many degrees of rest
constexpr double ARM_KP = 1.5;               // power (out of 127) per degree of error
constexpr double ARM_TOLERANCE = 3;          // degrees

// ============================================================================
// Pre-auto check
// ============================================================================

constexpr int BATTERY_MIN_PERCENT = 80;
constexpr double IMU_MAX_DISAGREE = 2;       // degrees between the two IMUs before the check turns red

} // namespace cfg

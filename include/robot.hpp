#pragma once

#include "api.h"
#include "lemlib/api.hpp"

#include <atomic>

// Devices and mechanisms of the rebuild, used by driver control and the autos.

/** Two IMUs that act like one: LemLib reads the average of both. If one stops working,
 * it uses the other one alone. */
class DualImu : public pros::Imu {
	public:
		DualImu(std::uint8_t portA, std::uint8_t portB);
		std::int32_t reset(bool blocking = false) const override;
		bool is_calibrating() const override;
		double get_rotation() const override;
		double get_heading() const override;
		std::int32_t set_rotation(double target) const override;
		std::int32_t set_heading(double target) const override;
		std::int32_t tare_rotation() const override;
		std::int32_t tare_heading() const override;
		/** Each IMU's own reading (infinity if it isn't working), for the check screen. */
		double rotationA() const;
		double rotationB() const;
		bool okA() const;
		bool okB() const;
	private:
		pros::Imu b;
};

extern pros::MotorGroup leftDrive, rightDrive;
extern pros::MotorGroup cascadeMotors; // the 2 that always drive the cascade
extern pros::Motor ptoMotor;            // the 3rd: cascade or intake
extern pros::Motor arm;
extern DualImu imu;
extern pros::Rotation verticalEnc, horizontalEnc, cascadeRot;
extern pros::Distance distLeft, distRight, distFront, clawDist;
extern pros::Optical toggleOptical, sideToggleOptical;
extern pros::adi::Pneumatics clawPiston, ptoPiston, armFlipPiston, balancerPiston;
extern lemlib::Chassis chassis;
extern pros::Controller controller;

/** Starts the background tasks (cascade, arm). Call once from initialize(). */
void robotStart();

/** Calibrates both IMUs and the odometry (robot must sit still ~3 s). */
void calibrate();

// ---- cascade ----
/** Claw rise in inches above all the way down (0 = down). NAN if not set up yet. */
double cascadeInches();
/** Bottom of the claw above the floor, inches. */
double clawHeight();
/** Starts moving the cascade so the claw rise is `inches`; returns right away. */
void cascadeTo(double inches);
/** Starts moving so the bottom of the claw is `inches` above the floor. */
void clawHeightTo(double inches);
/** Waits until the cascade gets there (or timeout ms). Returns true if it got there. */
bool cascadeWait(int timeout = 2000);
/** Driver control: drives the cascade at this power (-127..127) and stops any move to a height. */
void cascadeManual(int power);

/** Whether the balancers are out (they move by themselves with the cascade). */
bool balancersOut();

// ---- PTO / intake ----
/** Shifts the 3rd motor: true = intake, false = cascade. Waits for the gears to slide. */
void setIntakeMode(bool intake);
bool intakeMode();
/** Spins the intake (-127..127). Does nothing unless the PTO is on the intake. */
void intake(int power);

// ---- claw ----
void clawClose(bool wait = true);
void clawOpen(bool wait = true);
bool clawClosed();
/** Whether the claw distance sensor sees a pin in the claw. */
bool clawHasPin();

// ---- toggle arms ----
/** Arm angle from the hardstop, motor degrees (0 = where it was when the program started). */
double armAngle();
/** Starts moving the arm to this angle from rest; returns right away. */
void armTo(double deg);
void armOut();
void armIn();
bool armWait(int timeout = 1000);
/** Driver control: runs the arm at this power and stops any move to an angle. */
void armManual(int power);
/** The double-acting arm flip piston. */
void armFlip(bool out);
bool armFlipped();

// ---- toggle optical ----
enum class ToggleColor { NONE, RED, BLUE };
/** The color a toggle optical sees (NONE if nothing is close or the color isn't clear).
 * side = false: the first toggle optical, true: the side one. */
ToggleColor toggleColor(bool side = false);
/** Whether the toggle in front of that sensor shows our alliance's color. */
bool toggleIsOurs(bool side = false);
/** The hue that sensor reads (check screen). */
double toggleHue(bool side = false);

// ---- wall resets ----
/** Which walls a reset used (for the report). */
struct WallReset {
		bool x = false, y = false;
		float dx = 0, dy = 0; // how much it moved the robot
};
/** Fixes the robot's X and/or Y from the wall distance sensors, if they point square at a wall
 * and agree with odometry within a few inches. Robot should be still. */
WallReset wallReset();

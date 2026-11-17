#include "main.h"
#include "autos.hpp"
#include "config.hpp"
#include "robot.hpp"
#include "ui.hpp"

#include <cmath>

void initialize() {
	robotStart();
	ui::start();
	calibrate(); // keep the robot still for ~3 s after the program starts
}

void disabled() {}

void competition_initialize() {}

/** Runs the picked auto and shows the 30 s report after it. */
void autonomous() {
	std::uint32_t start = pros::millis();
	ui::clearReport();
	ui::report("%s  %s   build %d", AUTOS[selectedAuto].name, isRed ? "RED" : "BLUE", ui::BUILD);
	runSelectedAuto();
	lemlib::Pose p = chassis.getPose();
	ui::report("done in %.2f s", (pros::millis() - start) / 1000.0);
	ui::report("end: x %.1f  y %.1f  heading %.1f", p.x, p.y, p.theta);
	ui::report("IMUs differ %.1f deg, battery %d%%", std::abs(imu.rotationA() - imu.rotationB()),
	           static_cast<int>(pros::battery::get_capacity()));
	if (!std::isnan(cascadeInches())) ui::report("cascade %.1f in, claw %s", cascadeInches(), clawClosed() ? "closed" : "open");
	ui::showReport(30000);
}

// joystick movement smaller than this is ignored so the robot doesn't creep
constexpr int DEADBAND = 8;

/**
 * Driver controls (first try; tell us what you want changed):
 *   Left stick up/down: drive    Right stick left/right: turn
 *   R1 / R2: cascade up / down (holds when let go)    B: cascade all the way down
 *   L1: claw open / close
 *   L2 (hold): intake in    Down: intake out
 *   Up: toggle arms out / in    X: arm flip piston
 *   Off the field only: B + Down together runs the picked auto (with the report).
 */
void opcontrol() {
	bool armsOut = false;
	while (true) {
		int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		if (std::abs(throttle) < DEADBAND) throttle = 0;
		if (std::abs(turn) < DEADBAND) turn = 0;
		chassis.arcade(throttle, turn);

		// practice: run the auto off the field
		if (!pros::competition::is_connected() && controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) &&
		    controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			controller.rumble(".");
			pros::delay(1000);
			autonomous();
			continue;
		}

		// cascade
		bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		static bool wasManual = false;
		if (up || down) {
			cascadeManual(up ? 127 : -100);
			wasManual = true;
		} else if (wasManual) {
			cascadeManual(0); // hold here
			wasManual = false;
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) cascadeTo(0);

		// claw
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			if (clawClosed()) clawOpen(false);
			else clawClose(false);
		}

		// intake
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intake(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			intake(-127);
		} else {
			intake(0);
		}

		// arms
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			armsOut = !armsOut;
			if (armsOut) armOut();
			else armIn();
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) armFlip(!armFlipped());

		pros::delay(10);
	}
}

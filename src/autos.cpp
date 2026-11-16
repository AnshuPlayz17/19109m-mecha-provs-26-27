#include "autos.hpp"
#include "robot.hpp"
#include "field.hpp"
#include "config.hpp"
#include "ui.hpp"

// ============================================================================
// Helpers (red-side coordinates, mirrored on blue)
// ============================================================================

std::atomic<int> selectedAuto {0};
std::atomic<bool> isRed {true};

float mx(float x) { return isRed ? x : -x; }
float mh(float heading) { return isRed ? heading : -heading; }

Side mside(Side side) {
	if (isRed || side == Side::FRONT) return side;
	return side == Side::LEFT ? Side::RIGHT : Side::LEFT;
}

void goTo(float x, float y, int timeout, bool forwards, float maxSpeed) {
	chassis.moveToPoint(mx(x), y, timeout, {.forwards = forwards, .maxSpeed = maxSpeed});
}

void turnTo(float heading, int timeout) { chassis.turnToHeading(mh(heading), timeout); }

void face(float x, float y, int timeout, bool forwards) {
	chassis.turnToPoint(mx(x), y, timeout, {.forwards = forwards});
}

void waitMove() { chassis.waitUntilDone(); }

// ============================================================================
// The autos. Start from 0: the team adds them one at a time.
// ============================================================================

namespace {

void doNothing() {}

/** Test: drive 24" forward, then back. Shows how far odometry thinks it went. */
void driveTest() {
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, 24, 2000);
	waitMove();
	ui::report("after 24\" fwd: y %.2f", chassis.getPose().y);
	chassis.moveToPoint(0, 0, 2000, {.forwards = false});
	waitMove();
	ui::report("back to 0: y %.2f", chassis.getPose().y);
}

/** Test: turn 90° and back. */
void turnTest() {
	chassis.setPose(0, 0, 0);
	chassis.turnToHeading(90, 1000);
	waitMove();
	ui::report("turn to 90: %.2f", chassis.getPose().theta);
	chassis.turnToHeading(0, 1000);
	waitMove();
	ui::report("turn to 0: %.2f", chassis.getPose().theta);
}

} // namespace

const AutoEntry AUTOS[] = {
	{"0: Do nothing", doNothing, 0, 0, 0},
	{"T1: Drive test 24\"", driveTest, 0, 0, 0},
	{"T2: Turn test 90", turnTest, 0, 0, 0},
};
const int AUTO_COUNT = sizeof(AUTOS) / sizeof(AUTOS[0]);

void runSelectedAuto() {
	const AutoEntry& a = AUTOS[selectedAuto];
	chassis.setPose(mx(a.startX), a.startY, mh(a.startHeading));
	a.run();
}

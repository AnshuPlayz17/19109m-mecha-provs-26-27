#include "robot.hpp"
#include "config.hpp"
#include "field.hpp"

#include <algorithm>
#include <cmath>

using namespace cfg;

// ============================================================================
// Two IMUs averaged
// ============================================================================

namespace {
bool good(double v) { return std::isfinite(v) && v != PROS_ERR_F; }
} // namespace

DualImu::DualImu(std::uint8_t portA, std::uint8_t portB) : pros::Imu(portA), b(portB) {}

std::int32_t DualImu::reset(bool blocking) const {
	b.reset(false);
	std::int32_t r = pros::Imu::reset(false);
	if (blocking) {
		pros::delay(100);
		while (is_calibrating()) pros::delay(10);
	}
	return r;
}

bool DualImu::is_calibrating() const { return pros::Imu::is_calibrating() || b.is_calibrating(); }

double DualImu::rotationA() const {
	double v = pros::Imu::get_rotation();
	return good(v) ? v * IMU_A_SCALE : INFINITY;
}

double DualImu::rotationB() const {
	double v = b.get_rotation();
	return good(v) ? v * IMU_B_SCALE : INFINITY;
}

bool DualImu::okA() const { return std::isfinite(rotationA()); }
bool DualImu::okB() const { return std::isfinite(rotationB()); }

double DualImu::get_rotation() const {
	double ra = rotationA(), rb = rotationB();
	if (std::isfinite(ra) && std::isfinite(rb)) return (ra + rb) / 2;
	if (std::isfinite(ra)) return ra;
	if (std::isfinite(rb)) return rb;
	return PROS_ERR_F;
}

double DualImu::get_heading() const {
	double r = get_rotation();
	if (!good(r)) return PROS_ERR_F;
	double h = std::fmod(r, 360);
	return h < 0 ? h + 360 : h;
}

std::int32_t DualImu::set_rotation(double target) const {
	b.set_rotation(target / IMU_B_SCALE);
	return pros::Imu::set_rotation(target / IMU_A_SCALE);
}

std::int32_t DualImu::set_heading(double target) const { return set_rotation(target); }
std::int32_t DualImu::tare_rotation() const { return set_rotation(0); }
std::int32_t DualImu::tare_heading() const { return set_rotation(0); }

// ============================================================================
// Devices
// ============================================================================

pros::MotorGroup leftDrive({LEFT_FRONT, LEFT_BACK}, pros::MotorGears::blue);
pros::MotorGroup rightDrive({RIGHT_FRONT, RIGHT_BACK}, pros::MotorGears::blue);
pros::MotorGroup cascadeMotors({CASCADE_A, CASCADE_B});
pros::Motor ptoMotor(PTO_MOTOR);
pros::MotorGroup arms({ARM_LEFT, ARM_RIGHT});

DualImu imu(IMU_A, IMU_B);
pros::Rotation verticalEnc(TRACK_VERTICAL);
pros::Rotation horizontalEnc(TRACK_HORIZONTAL);
pros::Rotation armRot(ARM_ROT);
pros::Distance distLeft(DIST_LEFT), distRight(DIST_RIGHT), distBack(DIST_BACK), distFront(DIST_FRONT), clawDist(CLAW_DIST);
pros::Optical clawOptical(CLAW_OPTICAL);

pros::adi::Pneumatics clawPiston(CLAW_PISTON, false);
pros::adi::Pneumatics ptoPiston(PTO_PISTON, false);
pros::adi::Pneumatics armFlipPiston(ARM_FLIP_PISTON, false);
pros::adi::Pneumatics balancerPiston(BALANCER_PISTON, !BALANCER_OUT_IS_EXTENDED);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ============================================================================
// LemLib
// ============================================================================

lemlib::Drivetrain drivetrain(&leftDrive, &rightDrive, TRACK_WIDTH, lemlib::Omniwheel::NEW_275, DRIVE_RPM,
                              HORIZONTAL_DRIFT);
lemlib::TrackingWheel verticalWheel(&verticalEnc, TRACK_WHEEL_DIAMETER, VERTICAL_OFFSET);
lemlib::TrackingWheel horizontalWheel(&horizontalEnc, TRACK_WHEEL_DIAMETER, HORIZONTAL_OFFSET);
lemlib::OdomSensors sensors(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);

// Starting numbers. Tune at the robot (drive 24", turn 90°, watch the overshoot).
lemlib::ControllerSettings lateralPid(10, // kP
                                      0, // kI
                                      30, // kD
                                      3, // anti-windup range, in
                                      0.5, // small error range, in
                                      100, // small error timeout, ms
                                      1.5, // large error range, in
                                      400, // large error timeout, ms
                                      20); // slew
lemlib::ControllerSettings angularPid(2, // kP
                                      0, // kI
                                      10, // kD
                                      3, // anti-windup range, deg
                                      1, // small error range, deg
                                      100, // small error timeout, ms
                                      3, // large error range, deg
                                      400, // large error timeout, ms
                                      0); // slew

lemlib::Chassis chassis(drivetrain, lateralPid, angularPid, sensors);

void calibrate() {
	imu.reset(true);
	chassis.calibrate(false); // the IMUs are done; this resets the tracking wheels
}

// ============================================================================
// Cascade + PTO
// ============================================================================

namespace {
std::atomic<bool> cascadeAuto {false};
std::atomic<double> cascadeTarget {0};
std::atomic<int> cascadeManualPower {0};
std::atomic<bool> cascadeThere {false};
std::atomic<bool> inIntake {false};

bool cascadeSetUp() { return CASCADE_IN_PER_DEG != 0; }

/** Power to the cascade: the 2 cascade motors, plus the PTO motor if it's on the cascade. */
void cascadePower(int power) {
	if (power == 0) cascadeMotors.brake();
	else cascadeMotors.move(power);
	if (!inIntake) {
		if (power == 0) ptoMotor.brake();
		else ptoMotor.move(power);
	}
}

std::atomic<bool> balancersAreOut {false};

/** Balancers out at the top of the cascade, in again once it comes down. */
void updateBalancers() {
	double in = cascadeInches();
	bool out = balancersAreOut;
	if (std::isnan(in)) out = false;
	else if (in >= BALANCER_OUT_AT) out = true;
	else if (in < BALANCER_IN_AT) out = false;
	if (out == balancersAreOut) return;
	balancersAreOut = out;
	if (out == BALANCER_OUT_IS_EXTENDED) balancerPiston.extend();
	else balancerPiston.retract();
}

void cascadeTask() {
	while (true) {
		updateBalancers();
		if (cascadeAuto && cascadeSetUp()) {
			double error = cascadeTarget - cascadeInches();
			if (std::abs(error) < CASCADE_TOLERANCE) {
				cascadeThere = true;
				cascadePower(cascadeTarget > CASCADE_TOLERANCE ? CASCADE_HOLD_POWER : 0);
			} else {
				cascadeThere = false;
				int power = std::clamp(static_cast<int>(error * CASCADE_KP), -127, 127);
				if (std::abs(power) < CASCADE_MIN_POWER) power = error > 0 ? CASCADE_MIN_POWER : -CASCADE_MIN_POWER;
				cascadePower(power);
			}
		} else {
			cascadePower(cascadeManualPower);
		}
		pros::delay(10);
	}
}
} // namespace

double cascadeInches() {
	if (!cascadeSetUp()) return NAN;
	// the 2 cascade motors are always on the cascade: their encoders give the height (average of both)
	double a = cascadeMotors.get_position(0), b = cascadeMotors.get_position(1);
	if (!good(a) && !good(b)) return NAN; // both unplugged
	if (!good(a)) a = b;
	if (!good(b)) b = a;
	return (a + b) / 2 * CASCADE_IN_PER_DEG;
}

bool balancersOut() { return balancersAreOut; }

double clawHeight() { return CLAW_HEIGHT_DOWN + cascadeInches(); }

void cascadeTo(double inches) {
	cascadeTarget = std::clamp(inches, 0.0, CASCADE_TRAVEL);
	cascadeThere = false;
	cascadeAuto = true;
}

void clawHeightTo(double inches) { cascadeTo(inches - CLAW_HEIGHT_DOWN); }

bool cascadeWait(int timeout) {
	std::uint32_t start = pros::millis();
	pros::delay(20);
	while (!cascadeThere) {
		if (pros::millis() - start > static_cast<std::uint32_t>(timeout) || !cascadeSetUp()) return false;
		pros::delay(10);
	}
	return true;
}

void cascadeManual(int power) {
	cascadeAuto = false;
	cascadeManualPower = power;
}

void setIntakeMode(bool on) {
	if (on == inIntake) return;
	ptoMotor.brake();
	inIntake = on; // the cascade task stops (or starts) driving the PTO motor
	if (on == PTO_OUT_IS_INTAKE) ptoPiston.extend();
	else ptoPiston.retract();
	pros::delay(PTO_SHIFT_MS);
}

bool intakeMode() { return inIntake; }

void intake(int power) {
	if (!inIntake) return;
	if (power == 0) ptoMotor.brake();
	else ptoMotor.move(power);
}

// ============================================================================
// Claw
// ============================================================================

void clawClose(bool wait) {
	if (CLAW_OUT_IS_CLOSED) clawPiston.extend();
	else clawPiston.retract();
	if (wait) pros::delay(CLAW_MS);
}

void clawOpen(bool wait) {
	if (CLAW_OUT_IS_CLOSED) clawPiston.retract();
	else clawPiston.extend();
	if (wait) pros::delay(CLAW_MS);
}

bool clawClosed() { return clawPiston.is_extended() == CLAW_OUT_IS_CLOSED; }

bool clawHasPin() {
	if (CLAW_PIN_MM <= 0) return false;
	int mm = clawDist.get_distance();
	return mm > 0 && mm <= CLAW_PIN_MM;
}

// ============================================================================
// Toggle arms
// ============================================================================

namespace {
std::atomic<bool> armAuto {false};
std::atomic<double> armTarget {0};
std::atomic<int> armManualPower {0};
std::atomic<bool> armThere {false};

void armTask() {
	while (true) {
		if (armAuto) {
			double error = armTarget - armAngle();
			if (std::abs(error) < ARM_TOLERANCE) {
				armThere = true;
				arms.brake();
			} else {
				armThere = false;
				arms.move(std::clamp(static_cast<int>(error * ARM_KP), -127, 127));
			}
		} else if (armManualPower == 0) {
			arms.brake();
		} else {
			arms.move(armManualPower);
		}
		pros::delay(10);
	}
}
} // namespace

double armAngle() {
	// absolute angle, so it's right even after the program restarts; -180..180 from rest
	double d = armRot.get_angle() / 100.0 - ARM_REST_DEG;
	while (d > 180) d -= 360;
	while (d < -180) d += 360;
	return d;
}

bool armZeroed() { return std::abs(armAngle()) <= ARM_ZERO_TOLERANCE; }

void armTo(double deg) {
	armTarget = deg;
	armThere = false;
	armAuto = true;
}

void armOut() {
	double out = ARM_OUT_DEG - ARM_REST_DEG;
	if (out != 0) armTo(out);
}

void armIn() { armTo(0); }

bool armWait(int timeout) {
	std::uint32_t start = pros::millis();
	pros::delay(20);
	while (!armThere) {
		if (pros::millis() - start > static_cast<std::uint32_t>(timeout)) return false;
		pros::delay(10);
	}
	return true;
}

void armManual(int power) {
	armAuto = false;
	armManualPower = power;
}

void armFlip(bool out) {
	if (out) armFlipPiston.extend();
	else armFlipPiston.retract();
}

bool armFlipped() { return armFlipPiston.is_extended(); }

// ============================================================================
// Wall resets
// ============================================================================

namespace {
constexpr double DEG = M_PI / 180;

struct WallSensor {
		pros::Distance* sensor;
		float x, y;   // where it sits on the robot (x right, y forward)
		float facing; // which way it points, degrees from the robot's front (clockwise)
};

/** -180..180 */
double wrap(double a) {
	while (a > 180) a -= 360;
	while (a < -180) a += 360;
	return a;
}
} // namespace

WallReset wallReset() {
	WallSensor wallSensors[] = {
		{&distLeft, DIST_LEFT_X, DIST_LEFT_Y, -90},
		{&distRight, DIST_RIGHT_X, DIST_RIGHT_Y, 90},
		{&distBack, DIST_BACK_X, DIST_BACK_Y, 180},
		{&distFront, DIST_FRONT_X, DIST_FRONT_Y, 0},
	};
	lemlib::Pose pose = chassis.getPose();
	double t = pose.theta * DEG;
	WallReset result;
	double newX = pose.x, newY = pose.y;

	for (const WallSensor& w : wallSensors) {
		int mm = w.sensor->get_distance();
		if (mm <= 0 || mm > WALL_RESET_MAX_MM || mm == PROS_ERR) continue;
		double d = mm / 25.4;
		// where the sensor is on the field, from the robot's center (LemLib: heading 0 = +Y, clockwise)
		double sx = w.x * std::cos(t) + w.y * std::sin(t);
		double sy = -w.x * std::sin(t) + w.y * std::cos(t);
		double facing = wrap(pose.theta + w.facing);
		// which wall it points at, and how far off square
		for (int wall = 0; wall < 4; wall++) {
			double wallAngle = wall * 90.0; // 0 = far wall (+Y), 90 = right (+X), 180 = near (-Y), 270 = left (-X)
			double off = wrap(facing - wallAngle);
			if (std::abs(off) > WALL_RESET_MAX_ANGLE) continue;
			double along = d * std::cos(off * DEG); // straight-line distance to the wall
			if (wall == 0 || wall == 2) {
				double y = (wall == 0 ? field::WALL - along : -field::WALL + along) - sy;
				if (std::abs(y - pose.y) <= WALL_RESET_MAX_JUMP && !result.y) newY = y, result.y = true;
			} else {
				double x = (wall == 1 ? field::WALL - along : -field::WALL + along) - sx;
				if (std::abs(x - pose.x) <= WALL_RESET_MAX_JUMP && !result.x) newX = x, result.x = true;
			}
		}
	}
	result.dx = newX - pose.x;
	result.dy = newY - pose.y;
	if (result.x || result.y) chassis.setPose(newX, newY, pose.theta);
	return result;
}

// ============================================================================

void robotStart() {
	cascadeMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	ptoMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	arms.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	cascadeMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
	cascadeMotors.tare_position_all(); // the cascade starts all the way down
	clawOptical.set_led_pwm(100);
	pros::Task cascadeLoop(cascadeTask, "cascade");
	pros::Task armLoop(armTask, "arm");
}

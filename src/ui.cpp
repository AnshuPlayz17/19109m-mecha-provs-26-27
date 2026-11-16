#include "ui.hpp"
#include "autos.hpp"
#include "config.hpp"
#include "robot.hpp"
#include "liblvgl/lvgl.h"

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>

using namespace cfg;

namespace ui {

namespace {

constexpr std::uint32_t GREEN = 0x2ecc40, RED = 0xff4136, YELLOW = 0xffdc00, WHITE = 0xffffff, GREY = 0x333333;

// ---- the report (written by the auto, shown by the screen) ----
pros::Mutex reportLock;
std::string reportText;
std::atomic<std::uint32_t> reportUntil {0};
std::atomic<bool> reportShown {false};

// ---- checks ----
struct Check {
		std::string text;
		std::uint32_t color;
};

enum CheckId { C_IMU, C_PORTS, C_BATTERY, C_ARM, C_CASCADE, C_CLAW, C_TOGGLES, C_SETTINGS, C_AIR, C_COUNT };

struct Port {
		int port;
		pros::DeviceType type;
		const char* name;
};

const Port PORTS[] = {
	{std::abs(LEFT_FRONT), pros::DeviceType::motor, "LF"},
	{std::abs(LEFT_BACK), pros::DeviceType::motor, "LB"},
	{std::abs(RIGHT_FRONT), pros::DeviceType::motor, "RF"},
	{std::abs(RIGHT_BACK), pros::DeviceType::motor, "RB"},
	{std::abs(CASCADE_A), pros::DeviceType::motor, "CascA"},
	{std::abs(CASCADE_B), pros::DeviceType::motor, "CascB"},
	{std::abs(PTO_MOTOR), pros::DeviceType::motor, "PTO"},
	{std::abs(ARM_LEFT), pros::DeviceType::motor, "ArmL"},
	{std::abs(ARM_RIGHT), pros::DeviceType::motor, "ArmR"},
	{IMU_A, pros::DeviceType::imu, "IMU A"},
	{IMU_B, pros::DeviceType::imu, "IMU B"},
	{std::abs(TRACK_VERTICAL), pros::DeviceType::rotation, "TrackV"},
	{std::abs(TRACK_HORIZONTAL), pros::DeviceType::rotation, "TrackH"},
	{DIST_LEFT, pros::DeviceType::distance, "DistL"},
	{DIST_RIGHT, pros::DeviceType::distance, "DistR"},
	{DIST_FRONT, pros::DeviceType::distance, "DistF"},
	{CLAW_DIST, pros::DeviceType::distance, "ClawDist"},
	{std::abs(CASCADE_ROT), pros::DeviceType::rotation, "CascRot"},
	{TOGGLE_OPTICAL, pros::DeviceType::optical, "ToggleOpt"},
	{SIDE_TOGGLE_OPTICAL, pros::DeviceType::optical, "SideOpt"},
};

/** How many measurements in config.hpp are still placeholders. */
int unmeasured() {
	double values[] = {CASCADE_IN_PER_DEG, CLAW_HEIGHT_DOWN, static_cast<double>(CLAW_PIN_MM),
	                   ARM_OUT_DEG};
	int n = 0;
	for (double v : values)
		if (v == 0) n++;
	return n;
}

char buf[160];

Check checkImu() {
	bool calibrating = imu.is_calibrating();
	bool a = imu.okA(), b = imu.okB();
	if (calibrating) return {"IMUs: calibrating, keep still", YELLOW};
	if (!a || !b) return {std::string("IMU ") + (!a ? "A" : "") + (!a && !b ? "+" : "") + (!b ? "B" : "") + " not working", RED};
	double diff = std::abs(imu.rotationA() - imu.rotationB());
	std::snprintf(buf, sizeof buf, "IMUs ok, differ %.1f deg, heading %.1f", diff, imu.get_rotation());
	return {buf, diff <= IMU_MAX_DISAGREE ? GREEN : RED};
}

Check checkPorts() {
	std::string missing;
	for (const Port& p : PORTS)
		if (pros::Device::get_plugged_type(p.port) != p.type)
			missing += std::string(missing.empty() ? "" : ", ") + std::to_string(p.port) + " " + p.name;
	if (missing.empty()) return {"All 20 devices plugged in", GREEN};
	return {"Unplugged: " + missing, RED};
}

Check checkBattery() {
	int pct = static_cast<int>(pros::battery::get_capacity());
	std::snprintf(buf, sizeof buf, "Battery %d%%", pct);
	return {buf, pct >= BATTERY_MIN_PERCENT ? GREEN : RED};
}

Check checkArm() {
	if (armFlipped()) return {"Arm flip is OUT: put it in", RED};
	std::snprintf(buf, sizeof buf, "Arms on the hardstop? (%.0f deg since start)", armAngle());
	return {buf, YELLOW};
}

Check checkCascade() {
	double in = cascadeInches();
	if (std::isnan(in)) return {"Cascade: not set up (CASCADE_IN_PER_DEG)", YELLOW};
	std::snprintf(buf, sizeof buf, "Cascade %.1f in up (%.0f deg), PTO on %s%s", in,
	              cascadeRot.get_position() / 100.0, intakeMode() ? "intake" : "cascade",
	              balancersOut() ? ", BALANCERS OUT" : "");
	return {buf, std::abs(in) < 0.5 && !balancersOut() ? GREEN : RED};
}

Check checkClaw() {
	std::snprintf(buf, sizeof buf, "Claw %s, %s (%d mm)", clawClosed() ? "closed" : "open",
	              clawHasPin() ? "pin in" : "no pin", static_cast<int>(clawDist.get_distance()));
	return {buf, WHITE};
}

const char* colorName(ToggleColor c) { return c == ToggleColor::RED ? "RED" : c == ToggleColor::BLUE ? "BLUE" : "-"; }

Check checkToggles() {
	std::snprintf(buf, sizeof buf, "Toggle optical %s (hue %.0f), side %s (hue %.0f)", colorName(toggleColor(false)),
	              toggleHue(false), colorName(toggleColor(true)), toggleHue(true));
	return {buf, WHITE};
}

Check checkSettings() {
	int n = unmeasured();
	if (n == 0) return {"All measurements filled in", GREEN};
	std::snprintf(buf, sizeof buf, "%d measurements still MEASURE in config.hpp", n);
	return {buf, RED};
}

Check checkAir() { return {"Air: check the gauge", YELLOW}; }

Check runCheck(int i) {
	switch (i) {
		case C_IMU: return checkImu();
		case C_PORTS: return checkPorts();
		case C_BATTERY: return checkBattery();
		case C_ARM: return checkArm();
		case C_CASCADE: return checkCascade();
		case C_CLAW: return checkClaw();
		case C_TOGGLES: return checkToggles();
		case C_SETTINGS: return checkSettings();
		default: return checkAir();
	}
}

std::atomic<bool> good {false};

// ---- screen objects ----
lv_obj_t* banner;
lv_obj_t* checkLabels[C_COUNT];
lv_obj_t* autoLabel;
lv_obj_t* allianceButton;
lv_obj_t* allianceLabel;
lv_obj_t* poseLabel;
lv_obj_t* reportBox;
lv_obj_t* reportLabel;
lv_obj_t* reportTimer;

void color(lv_obj_t* label, std::uint32_t c) { lv_obj_set_style_text_color(label, lv_color_hex(c), 0); }

lv_obj_t* label(lv_obj_t* parent, int x, int y, const lv_font_t* font, std::uint32_t c = WHITE) {
	lv_obj_t* l = lv_label_create(parent);
	lv_obj_set_pos(l, x, y);
	lv_obj_set_style_text_font(l, font, 0);
	color(l, c);
	lv_label_set_text(l, "");
	return l;
}

lv_obj_t* button(lv_obj_t* parent, int x, int y, int w, int h, const char* text, lv_event_cb_t cb) {
	lv_obj_t* b = lv_button_create(parent);
	lv_obj_set_pos(b, x, y);
	lv_obj_set_size(b, w, h);
	lv_obj_set_style_bg_color(b, lv_color_hex(GREY), 0);
	lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
	lv_obj_t* l = lv_label_create(b);
	lv_label_set_text(l, text);
	lv_obj_set_style_text_font(l, &lv_font_montserrat_20, 0);
	lv_obj_center(l);
	return b;
}

bool canPick() { return pros::competition::is_disabled() || !pros::competition::is_connected(); }

void prevAuto(lv_event_t*) {
	if (canPick()) selectedAuto = (selectedAuto + AUTO_COUNT - 1) % AUTO_COUNT;
}
void nextAuto(lv_event_t*) {
	if (canPick()) selectedAuto = (selectedAuto + 1) % AUTO_COUNT;
}
void flipAlliance(lv_event_t*) {
	if (canPick()) isRed = !isRed;
}
void closeReport(lv_event_t*) { reportUntil = 0; }

void refresh(lv_timer_t*) {
	// report
	std::uint32_t now = pros::millis();
	bool showing = reportUntil > now;
	if (showing) {
		reportLock.take();
		lv_label_set_text(reportLabel, reportText.c_str());
		reportLock.give();
		std::snprintf(buf, sizeof buf, "%lu s  (tap to close)", static_cast<unsigned long>((reportUntil - now) / 1000 + 1));
		lv_label_set_text(reportTimer, buf);
	}
	if (showing != reportShown) {
		if (showing) lv_obj_remove_flag(reportBox, LV_OBJ_FLAG_HIDDEN);
		else lv_obj_add_flag(reportBox, LV_OBJ_FLAG_HIDDEN);
		reportShown = showing;
	}

	// checks
	bool allGreen = true;
	for (int i = 0; i < C_COUNT; i++) {
		Check c = runCheck(i);
		if (c.color == RED) allGreen = false;
		lv_label_set_text(checkLabels[i], c.text.c_str());
		color(checkLabels[i], c.color);
	}
	good = allGreen;
	lv_label_set_text(banner, allGreen ? "READY" : "NOT READY");
	color(banner, allGreen ? GREEN : RED);

	lv_label_set_text(autoLabel, AUTOS[selectedAuto].name);
	lv_label_set_text(allianceLabel, isRed ? "RED" : "BLUE");
	lv_obj_set_style_bg_color(allianceButton, lv_color_hex(isRed ? 0xc0392b : 0x2980b9), 0);

	lemlib::Pose p = chassis.getPose();
	std::snprintf(buf, sizeof buf, "x %.1f  y %.1f  h %.1f", p.x, p.y, p.theta);
	lv_label_set_text(poseLabel, buf);
}

void build() {
	lv_obj_t* scr = lv_screen_active();
	lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

	lv_obj_t* title = label(scr, 6, 4, &lv_font_montserrat_16);
	std::snprintf(buf, sizeof buf, "19109M  build %d", BUILD);
	lv_label_set_text(title, buf);
	banner = label(scr, 6, 24, &lv_font_montserrat_24);

	for (int i = 0; i < C_COUNT; i++) {
		checkLabels[i] = label(scr, 6, 54 + i * 20, &lv_font_montserrat_14);
		lv_obj_set_width(checkLabels[i], 300);
		lv_label_set_long_mode(checkLabels[i], LV_LABEL_LONG_DOT);
	}

	// auto picker on the right
	autoLabel = label(scr, 316, 8, &lv_font_montserrat_16);
	lv_obj_set_width(autoLabel, 160);
	lv_label_set_long_mode(autoLabel, LV_LABEL_LONG_WRAP);
	button(scr, 316, 60, 75, 50, "<", prevAuto);
	button(scr, 399, 60, 75, 50, ">", nextAuto);
	allianceButton = button(scr, 316, 118, 158, 50, "", flipAlliance);
	allianceLabel = lv_obj_get_child(allianceButton, 0);
	poseLabel = label(scr, 316, 180, &lv_font_montserrat_12);
	lv_obj_set_width(poseLabel, 160);

	// the report, full screen on top
	reportBox = lv_obj_create(scr);
	lv_obj_set_pos(reportBox, 0, 0);
	lv_obj_set_size(reportBox, 480, 240);
	lv_obj_set_style_bg_color(reportBox, lv_color_hex(0x101820), 0);
	lv_obj_set_style_border_width(reportBox, 0, 0);
	lv_obj_set_style_radius(reportBox, 0, 0);
	lv_obj_remove_flag(reportBox, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_event_cb(reportBox, closeReport, LV_EVENT_CLICKED, nullptr);
	reportLabel = label(reportBox, 0, 0, &lv_font_montserrat_14);
	lv_obj_set_width(reportLabel, 440);
	reportTimer = label(reportBox, 300, 196, &lv_font_montserrat_12, YELLOW);
	lv_obj_add_flag(reportBox, LV_OBJ_FLAG_HIDDEN);

	lv_timer_create(refresh, 200, nullptr);
}

/** Controller: the build at boot, then the picked auto; arrows / A pick while disabled. */
void controllerTask() {
	std::snprintf(buf, sizeof buf, "19109M build %d   ", BUILD);
	controller.print(0, 0, "%s", buf);
	pros::delay(3000);
	int line = 0;
	while (true) {
		if (canPick()) {
			if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
				selectedAuto = (selectedAuto + AUTO_COUNT - 1) % AUTO_COUNT;
			if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
				selectedAuto = (selectedAuto + 1) % AUTO_COUNT;
			if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) isRed = !isRed;
		}
		// one line every 60 ms (the controller can't take more)
		char text[20];
		if (line == 0) std::snprintf(text, sizeof text, "%-19s", AUTOS[selectedAuto].name);
		else if (line == 1)
			std::snprintf(text, sizeof text, "%-4s bat %3d%% B%-4d", isRed ? "RED" : "BLUE",
			              static_cast<int>(pros::battery::get_capacity()), BUILD);
		else std::snprintf(text, sizeof text, "%-19s", good ? "READY" : "NOT READY");
		controller.print(line, 0, "%s", text);
		line = (line + 1) % 3;
		pros::delay(60);
	}
}

} // namespace

void start() {
	build();
	pros::Task ctrl(controllerTask, "controller");
}

void report(const char* fmt, ...) {
	char line[96];
	va_list args;
	va_start(args, fmt);
	std::vsnprintf(line, sizeof line, fmt, args);
	va_end(args);
	reportLock.take();
	reportText += line;
	reportText += "\n";
	reportLock.give();
}

void clearReport() {
	reportLock.take();
	reportText.clear();
	reportLock.give();
}

void showReport(int ms) { reportUntil = pros::millis() + ms; }

bool allGood() { return good; }

} // namespace ui

#pragma once

// The brain screen: the pre-auto check screen (with the auto picker), and the 30 s report after
// every auto. Plus the controller screen.
namespace ui {

/** The build number. Bump it with every change: the controller shows it at boot. */
constexpr int BUILD = 11;

/** Builds the screen and starts the controller task (call once, from initialize()). */
void start();

/** Adds a line to the auto report (printf style). Cleared when an auto starts. */
void report(const char* fmt, ...);
/** Clears the report (called when an auto starts). */
void clearReport();
/** Shows the report full screen for `ms` (tap to close). */
void showReport(int ms = 30000);

/** Whether every check on the check screen is green. */
bool allGood();

} // namespace ui

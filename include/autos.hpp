#pragma once

#include <atomic>

// The autos. Each one is written for the RED side; on BLUE it runs mirrored (x flipped).
// Add new autos to the list in src/autos.cpp.

struct AutoEntry {
		const char* name;      // short, fits on the controller (19 characters)
		void (*run)();
		float startX, startY, startHeading; // where the robot starts, red side (field inches, degrees)
};

extern const AutoEntry AUTOS[];
extern const int AUTO_COUNT;

/** Which auto is picked (index into AUTOS). */
extern std::atomic<int> selectedAuto;
/** true = red, false = blue. */
extern std::atomic<bool> isRed;

/** Runs the picked auto (sets the start position first). */
void runSelectedAuto();

// ---- helpers for writing autos (mirror automatically on blue) ----
/** x on our side: as written on red, flipped on blue. */
float mx(float x);
/** a heading on our side. */
float mh(float heading);
/** Drive to a point (red-side coordinates). */
void goTo(float x, float y, int timeout = 2000, bool forwards = true, float maxSpeed = 127);
/** Turn to a heading (red-side). */
void turnTo(float heading, int timeout = 1000);
/** Turn to face a point (red-side). */
void face(float x, float y, int timeout = 1000, bool forwards = true);
/** Waits until the current move is done. */
void waitMove();

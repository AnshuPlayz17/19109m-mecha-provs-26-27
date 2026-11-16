#pragma once

/**
 * Override field element positions, for planning autonomous routes.
 *
 * Origin at field center, +X right, +Y up in the top-down field image, inches.
 * Positions are read off the top-down field image and snapped to the tile grid,
 * so check them against the real field before relying on them.
 */
namespace field {

constexpr float TILE = 23.8; // one field tile, inches
constexpr float WALL = 3 * TILE; // distance from center to the inside of each wall

struct Point {
		float x;
		float y;
};

// neutral tall goal, center of the field
constexpr Point TALL_GOAL {0, 0};

// neutral short goals
constexpr Point SHORT_GOAL_TOP_LEFT {-TILE, 2 * TILE};
constexpr Point SHORT_GOAL_LEFT {-2 * TILE, TILE};
constexpr Point SHORT_GOAL_RIGHT {2 * TILE, -TILE};
constexpr Point SHORT_GOAL_BOTTOM_RIGHT {TILE, -2 * TILE};

// alliance goals, numbered as in the field image
constexpr Point RED_GOAL_2 {-2 * TILE, -TILE};
constexpr Point RED_GOAL_3 {-TILE, -2 * TILE};
constexpr Point BLUE_GOAL_2 {2 * TILE, TILE};
constexpr Point BLUE_GOAL_3 {TILE, 2 * TILE};

// toggles, at the center of each wall
constexpr Point TOGGLE_LEFT {-WALL, 0};
constexpr Point TOGGLE_RIGHT {WALL, 0};
constexpr Point TOGGLE_TOP {0, WALL};
constexpr Point TOGGLE_BOTTOM {0, -WALL};

// midfield: the diamond around the tall goal, corners one tile from center
constexpr Point MIDFIELD_LEFT {-TILE, 0};
constexpr Point MIDFIELD_RIGHT {TILE, 0};
constexpr Point MIDFIELD_TOP {0, TILE};
constexpr Point MIDFIELD_BOTTOM {0, -TILE};

// cross-shaped game object clusters on the long diagonal
constexpr Point CLUSTER_TOP_LEFT {-2 * TILE, 2 * TILE};
constexpr Point CLUSTER_LEFT {-TILE, TILE};
constexpr Point CLUSTER_RIGHT {TILE, -TILE};
constexpr Point CLUSTER_BOTTOM_RIGHT {2 * TILE, -2 * TILE};

// single yellow pins on the other diagonal
constexpr Point PIN_TOP_RIGHT {2 * TILE, 2 * TILE};
constexpr Point PIN_RIGHT {TILE, TILE};
constexpr Point PIN_LEFT {-TILE, -TILE};
constexpr Point PIN_BOTTOM_LEFT {-2 * TILE, -2 * TILE};

// ---- heights (inches) ----
constexpr float PIN_HEIGHT = 6.50;   // a pin nests 0.64" into the one under it
constexpr float CUP_HEIGHT = 6.48;
constexpr float ALLIANCE_GOAL_HEIGHT = 3.25;
constexpr float SHORT_GOAL_HEIGHT = 5.77;
constexpr float TALL_GOAL_HEIGHT = 8.77;
constexpr float FIRST_PIN_ADDS = 5.9;  // the first pin on a goal
constexpr float NEXT_STACK_ADDS = 6.5; // each pin + cup after that

/** Height of the top of a goal with `stacks` already on it (0 = empty goal). */
constexpr float stackTop(float goalHeight, int stacks) {
	return stacks <= 0 ? goalHeight : goalHeight + FIRST_PIN_ADDS + NEXT_STACK_ADDS * (stacks - 1);
}

} // namespace field

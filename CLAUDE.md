# 19109M rebuild (Override 2026-27)

PROS 4.2.2, LemLib 0.5.6, liblvgl 9.2. Code for the REBUILT robot (target early Feb).
The old robot's code (with autos 7 and 8 locked at build 355) stays in `19109M-Regionals-26-27`; nothing here touches it.

## Field
- 1 tile = 23.8 in. Origin at field center, +X right, +Y away from the driver. Positions and heights in `include/field.hpp`.
- Autos are written for RED; on BLUE they run mirrored (use `goTo`, `turnTo`, `face`, `mx`, `mh` in `src/autos.cpp`).

## Code layout
- `include/config.hpp`: every port, measurement and tuning number. `MEASURE` = placeholder, fill in from the real robot; never guess.
- `src/robot.cpp`: devices, the two IMUs averaged (`DualImu`), LemLib, cascade, intake, claw, toggle arms, wall resets.
- `src/autos.cpp`: the auto list. Started from 0; the team adds autos one at a time.
- `src/ui.cpp`: pre-auto check screen + auto picker, the 30 s report after every auto, the controller screen.

## Workflow
- Bump `ui::BUILD` in `include/ui.hpp` with every change, and end every chat message with it (e.g. **Build 2**).
- Run `make` to check it builds. Tell the team `git pull` + `pros mu`.
- After every auto test, ask for a photo of the 30 s report on the brain.

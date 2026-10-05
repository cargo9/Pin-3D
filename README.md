# Pin 3D

A small 3D physics sandbox for school-level mechanics experiments, written in C++20 with [raylib](https://www.raylib.com/).

Drop, throw, push, hang and link bodies, then watch what happens. Every value on screen is computed from real units (metres, kilograms, seconds), so the sandbox can be used to check the answers to typical physics problems: free fall, projectile motion, Newton's laws, friction on an inclined plane, momentum in collisions, pendulums, springs, pulleys and buoyancy.

## Features

- **Rigid bodies.** Spheres, cubes, boxes, pyramids, ellipsoids, cylinders, cones and capsules tumble, roll, topple and stack.
- **Materials** with real densities, restitution and friction: foam, wood, ice, plastic, rubber, clay, stone, steel, gold. You can also set the mass directly.
- **Liquids** in a glass tank: water, oil, honey, mercury. Includes Archimedes' buoyancy, drag, splashes and ripples.
- **World settings:** gravity presets (Earth, Moon, Mars, Jupiter, zero-g), slow motion from 0.1× to 2×, and custom or zero friction.
- **Weather:** wind with real air drag, and rain.
- **Lab tools:** an inclined plane with an adjustable angle, throwing from the camera, a constant push force, and drop height.
- **Joints:** ropes (pendulums), springs with stiffness k, rigid rods, and a two-wheel pulley (Atwood machine).
- **Measurements:**
  - a stopwatch;
  - a stats card for the selected body: speed, spin, height, max height, time, momentum, kinetic and potential energy, weight, buoyant force, pressure, spring stretch;
  - live graphs of height, speed and energy;
  - trajectories and velocity/force vectors.

## Build

Requires CMake 3.20+, a C++20 compiler and raylib.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/pin3d
```

## Controls

| Key | Action |
| --- | --- |
| LMB | Select, drag, throw |
| RMB + mouse | Look around |
| W A S D, E / C, Shift | Fly, up / down, faster |
| N | Spawn a body where you look |
| F | Throw a body from the camera |
| V / P | Give the selected body velocity / toggle a push force |
| H / J / K | Hang selected / link two bodies / cut joints |
| T (Shift+T) | Start/stop the stopwatch (reset) |
| Space / R | Pause / reset positions |
| Q / Del | Delete last (hold: all) / delete selected |
| Esc | Settings |

## Credits

Made with [Claude](https://claude.ai) AI by Anthropic.

The UI font is [Inter](https://rsms.me/inter/) by Rasmus Andersson, licensed under the SIL Open Font License (see `assets/fonts/Inter-LICENSE.txt`).

# Robot Path Planner

A small interactive 2D navigation simulator. Edit a grid, run a search, and watch a robot follow a shortest available route. This is a discrete grid simulation, not a physics or hardware controller.

**Author:** Azer Ben Ali

## Build and run

The project uses C++17, CMake 3.16 or newer, Git, and raylib 5.5. CMake downloads the pinned raylib release on the first configure, so that step needs an internet connection. On the development machine, the prerequisites are macOS 26.6, Apple Clang 21 and the macOS SDK from Xcode 26.6 (or the corresponding Xcode command-line tools), and CMake 4.4. Other platforms need a C++17 compiler and raylib's platform development dependencies.

From the project directory in a VS Code terminal:

```sh
cmake -S . -B build
cmake --build build
./build/robot_path_planner
```

## Controls

- Select **Obstacle** or **Erase**, then click and drag on the grid.
- Select **Start** or **Goal**, then click a cell. Endpoints cannot be placed in walls, and walls cannot be placed on endpoints.
- **Plan & Run** searches and starts the simulation. **Pause / Resume** freezes or continues it. **Reset Run** clears the current run while keeping the map.
- **Clear Walls** removes obstacles; **Load Demo** restores the original map. Choose a speed in the panel.
- Keyboard: **Enter** to plan and run, **Space** to pause or resume, and **R** to reset. Reset a running or paused simulation before editing the grid.

Breadth-first search (BFS) explores neighboring cells in a fixed order. Because each horizontal or vertical move has equal cost, the first route it finds is a shortest route. The visualization replays the actual expansion order before the robot travels along the route.

#include "PathPlanner.h"
#include <raylib.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace {

// Configuration Constants
constexpr int WindowWidth = 1280;
constexpr int WindowHeight = 800;
constexpr int Rows = 16;
constexpr int Cols = 20;
constexpr int CellSize = 36;
constexpr int GridX = 65;
constexpr int GridY = 132;

constexpr float PanelX = 830.0f;
constexpr float PanelWidth = 400.0f;
constexpr float ExploreRate = 35.0f;
constexpr float MoveRate = 3.0f;

constexpr std::array<float, 4> Speeds{0.5f, 1.0f, 2.0f, 4.0f};

// Color Palette
const Color Background{238, 242, 243, 255};
const Color Ink{31, 44, 52, 255};
const Color Muted{91, 110, 119, 255};
const Color Panel{255, 255, 255, 255};
const Color Empty{248, 249, 247, 255};
const Color Wall{48, 64, 72, 255};
const Color Line{194, 204, 205, 255};
const Color Explored{204, 226, 235, 255};
const Color Green{42, 145, 91, 255};
const Color Amber{225, 157, 48, 255};
const Color Blue{39, 109, 148, 255};
const Color Route{40, 121, 163, 255};

// Enums & Structs
enum class Tool { Wall, Erase, Start, Goal };
enum class Phase { Ready, Exploring, Moving, Paused, Completed, NoRoute };

struct App {
    Grid grid = Grid(Rows, std::vector<bool>(Cols, false));
    GridCoord start{2, 2};
    GridCoord goal{13, 17};
    Tool tool = Tool::Wall;
    Phase phase = Phase::Ready;
    Phase pausedPhase = Phase::Ready;
    SearchResult search{};
    std::size_t revealed = 0;
    std::size_t segment = 0;
    float exploreProgress = 0.0f;
    float segmentProgress = 0.0f;
    int speedIndex = 1;
    bool gridDrag = false;
    std::string message = "Choose a tool, then edit the grid.";
};

// Helper Functions
bool same(GridCoord a, GridCoord b) { 
    return a.row == b.row && a.col == b.col; 
}

bool active(const App& a) { 
    return a.phase == Phase::Exploring || a.phase == Phase::Moving || a.phase == Phase::Paused; 
}

Vector2 center(GridCoord p) { 
    return {GridX + (p.col + 0.5f) * CellSize, GridY + (p.row + 0.5f) * CellSize}; 
}

bool gridCell(Vector2 mouse, GridCoord& out) {
    if (mouse.x < GridX || mouse.y < GridY || mouse.x >= GridX + Cols * CellSize || mouse.y >= GridY + Rows * CellSize) {
        return false;
    }
    out = {static_cast<int>((mouse.y - GridY) / CellSize), static_cast<int>((mouse.x - GridX) / CellSize)};
    return true;
}

const char* toolName(Tool t) {
    switch (t) { 
        case Tool::Wall: return "Place obstacle"; 
        case Tool::Erase: return "Erase obstacle"; 
        case Tool::Start: return "Set start"; 
        case Tool::Goal: return "Set goal"; 
    }
    return "";
}

const char* phaseName(Phase p) {
    switch (p) { 
        case Phase::Ready: return "Ready to plan"; 
        case Phase::Exploring: return "Exploring"; 
        case Phase::Moving: return "Robot moving"; 
        case Phase::Paused: return "Paused"; 
        case Phase::Completed: return "Route complete"; 
        case Phase::NoRoute: return "No route available"; 
    }
    return "";
}
   / Simulation Control
void resetRun(App& a) {
    a.phase = Phase::Ready; 
    a.pausedPhase = Phase::Ready; 
    a.search = SearchResult{};
    a.revealed = 0;
    a.segment = 0; 
    a.exploreProgress = 0.0f;
    a.segmentProgress = 0.0f;
    a.message = "Ready to edit or plan a route.";
}

void loadDemo(App& a) {
    a.grid = Grid(Rows, std::vector<bool>(Cols, false));
    // Alternating gaps force a route with turns without obscuring the robot.
    for (int r = 0; r < Rows; ++r) {
        if (r != 11 && r != 12) a.grid[r][5] = true;
        if (r != 3 && r != 4) a.grid[r][10] = true;
        if (r != 11 && r != 12) a.grid[r][15] = true;
    }
    a.start = {2, 2}; 
    a.goal = {13, 17}; 
    resetRun(a);
    a.message = "Demo loaded. Select Plan & Run.";
}

void plan(App& a) {
    resetRun(a);
    a.search = PathPlanner::findPath(a.grid, a.start, a.goal);
    if (a.search.status == SearchStatus::InvalidEndpoints) { 
        a.message = "Start or goal is invalid."; 
        return; 
    }
    a.phase = Phase::Exploring;
    a.message = "Replaying BFS expansion order.";
}

void pauseResume(App& a) {
    if (a.phase == Phase::Paused) { 
        a.phase = a.pausedPhase; 
        a.message = "Simulation resumed."; 
    } else if (a.phase == Phase::Exploring || a.phase == Phase::Moving) { 
        a.pausedPhase = a.phase; 
        a.phase = Phase::Paused; 
        a.message = "Paused. Resume or reset to edit."; 
    }
}

void update(App& a, float dt) {
    if (a.phase == Phase::Exploring) {
        a.exploreProgress += dt * ExploreRate * Speeds[a.speedIndex];
        while (a.exploreProgress >= 1.0f && a.revealed < a.search.expansionOrder.size()) { 
            ++a.revealed; 
            a.exploreProgress -= 1.0f; 
        }
        if (a.revealed == a.search.expansionOrder.size()) {
            if (a.search.status == SearchStatus::NoRoute) { 
                a.phase = Phase::NoRoute; 
                a.message = "No route. Edit the map and retry."; 
            } else if (a.search.route.size() <= 1) { 
                a.phase = Phase::Completed; 
                a.message = "Already at the destination: 0 moves."; 
            } else { 
                a.phase = Phase::Moving; 
                a.message = "Following the shortest route."; 
            }
        }
    } else if (a.phase == Phase::Moving) {
        a.segmentProgress += dt * MoveRate * Speeds[a.speedIndex];
        while (a.segmentProgress >= 1.0f && a.segment + 1 < a.search.route.size()) {
            a.segmentProgress -= 1.0f; 
            ++a.segment;
        }
        if (a.segment + 1 >= a.search.route.size()) { 
            a.segmentProgress = 0; 
            a.phase = Phase::Completed; 
            a.message = "Destination reached."; 
        }
    }
}

Vector2 robotPosition(const App& a) {
    if (a.search.route.empty() || (a.phase != Phase::Moving && a.phase != Phase::Paused && a.phase != Phase::Completed)) {
        return center(a.start);
    }
    const auto index = std::min(a.segment, a.search.route.size() - 1);
    Vector2 p = center(a.search.route[index]);
    if (index + 1 < a.search.route.size()) { 
        Vector2 q = center(a.search.route[index + 1]); 
        p.x += (q.x - p.x) * a.segmentProgress; 
        p.y += (q.y - p.y) * a.segmentProgress; 
    }
    return p;
}

void editCell(App& a, GridCoord p) {
    if (a.tool == Tool::Wall) {
        if (same(p, a.start) || same(p, a.goal)) { 
            a.message = "Endpoints cannot be obstacles."; 
            return; 
        }
        if (!a.grid[p.row][p.col]) { 
            resetRun(a); 
            a.grid[p.row][p.col] = true; 
            a.message = "Obstacle placed."; 
        }
    } else if (a.tool == Tool::Erase) {
        if (a.grid[p.row][p.col]) { 
            resetRun(a); 
            a.grid[p.row][p.col] = false; 
            a.message = "Obstacle erased."; 
        }
    } else {
        if (a.grid[p.row][p.col]) { 
            a.message = "Choose an empty endpoint cell."; 
            return; 
        }
        GridCoord& endpoint = (a.tool == Tool::Start) ? a.start : a.goal;
        if (!same(endpoint, p)) { 
            resetRun(a); 
            endpoint = p; 
            a.message = "Endpoint updated."; 
        }
    }
}
        // Discard unfocused time and cap frame deltas to prevent large jumps.
        float dt = IsWindowFocused() ? std::min(GetFrameTime(), 0.1f) : 0.0f;
        update(app, dt);
        BeginDrawing(); ClearBackground(Background);
        text("ROBOT PATH PLANNER", 65, 33, 30, Ink);
        text("A shortest-path simulation on a 20 x 16 grid", 65, 72, 18, Muted);
        drawGrid(app); drawPanel(app);
        text("S  Start     G  Goal     Dark  Obstacle     Blue  Explored     Line  Route", 65, 730, 16, Ink);
        text("Drag to place or erase walls. Click once to set an endpoint.", 65, 757, 16, Muted);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}

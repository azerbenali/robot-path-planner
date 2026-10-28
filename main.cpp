#include "PathPlanner.h"
#include <raylib.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace {
constexpr int WindowWidth = 1280, WindowHeight = 800;
constexpr int Rows = 16, Cols = 20, CellSize = 36, GridX = 65, GridY = 132;
constexpr float PanelX = 830.0f, PanelWidth = 400.0f;
constexpr float ExploreRate = 35.0f, MoveRate = 3.0f;
constexpr std::array<float, 4> Speeds{0.5f, 1.0f, 2.0f, 4.0f};
const Color Background{238, 242, 243, 255}, Ink{31, 44, 52, 255};
const Color Muted{91, 110, 119, 255}, Panel{255, 255, 255, 255};
const Color Empty{248, 249, 247, 255}, Wall{48, 64, 72, 255};
const Color Line{194, 204, 205, 255}, Explored{204, 226, 235, 255};
const Color Green{42, 145, 91, 255}, Amber{225, 157, 48, 255};
const Color Blue{39, 109, 148, 255}, Route{40, 121, 163, 255};

enum class Tool { Wall, Erase, Start, Goal };
enum class Phase { Ready, Exploring, Moving, Paused, Completed, NoRoute };

struct App {
    Grid grid = Grid(Rows, std::vector<bool>(Cols, false));
    GridCoord start{2, 2}, goal{13, 17};
    Tool tool = Tool::Wall;
    Phase phase = Phase::Ready, pausedPhase = Phase::Ready;
    SearchResult search{};
    std::size_t revealed = 0, segment = 0;
    float exploreProgress = 0.0f, segmentProgress = 0.0f;
    int speedIndex = 1;
    bool gridDrag = false;
    std::string message = "Choose a tool, then edit the grid.";
};

bool same(GridCoord a, GridCoord b) { return a.row == b.row && a.col == b.col; }
bool active(const App& a) { return a.phase == Phase::Exploring || a.phase == Phase::Moving || a.phase == Phase::Paused; }
Vector2 center(GridCoord p) { return {GridX + (p.col + 0.5f)*CellSize, GridY + (p.row + 0.5f)*CellSize}; }
bool gridCell(Vector2 mouse, GridCoord& out) {
    if (mouse.x < GridX || mouse.y < GridY || mouse.x >= GridX + Cols*CellSize || mouse.y >= GridY + Rows*CellSize) return false;
    out = {static_cast<int>((mouse.y - GridY)/CellSize), static_cast<int>((mouse.x - GridX)/CellSize)};
    return true;
}
const char* toolName(Tool t) {
    switch (t) { case Tool::Wall: return "Place obstacle"; case Tool::Erase: return "Erase obstacle"; case Tool::Start: return "Set start"; case Tool::Goal: return "Set goal"; }
    return "";
}
const char* phaseName(Phase p) {
    switch (p) { case Phase::Ready: return "Ready to plan"; case Phase::Exploring: return "Exploring"; case Phase::Moving: return "Robot moving"; case Phase::Paused: return "Paused"; case Phase::Completed: return "Route complete"; case Phase::NoRoute: return "No route available"; }
    return "";
}
void resetRun(App& a) {
    a.phase = Phase::Ready; a.pausedPhase = Phase::Ready; a.search = SearchResult{};
    a.revealed = a.segment = 0; a.exploreProgress = a.segmentProgress = 0.0f;
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
    a.start = {2, 2}; a.goal = {13, 17}; resetRun(a);
    a.message = "Demo loaded. Select Plan & Run.";
}
void plan(App& a) {
    resetRun(a);
    a.search = PathPlanner::findPath(a.grid, a.start, a.goal);
    if (a.search.status == SearchStatus::InvalidEndpoints) { a.message = "Start or goal is invalid."; return; }
    a.phase = Phase::Exploring;
    a.message = "Replaying BFS expansion order.";
}
void pauseResume(App& a) {
    if (a.phase == Phase::Paused) { a.phase = a.pausedPhase; a.message = "Simulation resumed."; }
    else if (a.phase == Phase::Exploring || a.phase == Phase::Moving) { a.pausedPhase = a.phase; a.phase = Phase::Paused; a.message = "Paused. Resume or reset to edit."; }
}
void update(App& a, float dt) {
    if (a.phase == Phase::Exploring) {
        a.exploreProgress += dt * ExploreRate * Speeds[a.speedIndex];
        while (a.exploreProgress >= 1.0f && a.revealed < a.search.expansionOrder.size()) { ++a.revealed; a.exploreProgress -= 1.0f; }
        if (a.revealed == a.search.expansionOrder.size()) {
            if (a.search.status == SearchStatus::NoRoute) { a.phase = Phase::NoRoute; a.message = "No route. Edit the map and retry."; }
            else if (a.search.route.size() <= 1) { a.phase = Phase::Completed; a.message = "Already at the destination: 0 moves."; }
            else { a.phase = Phase::Moving; a.message = "Following the shortest route."; }
        }
    } else if (a.phase == Phase::Moving) {
        a.segmentProgress += dt * MoveRate * Speeds[a.speedIndex];
        while (a.segmentProgress >= 1.0f && a.segment + 1 < a.search.route.size()) {
            a.segmentProgress -= 1.0f; ++a.segment;
        }
        if (a.segment + 1 >= a.search.route.size()) { a.segmentProgress = 0; a.phase = Phase::Completed; a.message = "Destination reached."; }
    }
}
Vector2 robotPosition(const App& a) {
    if (a.search.route.empty() || (a.phase != Phase::Moving && a.phase != Phase::Paused && a.phase != Phase::Completed)) return center(a.start);
    const auto index = std::min(a.segment, a.search.route.size() - 1);
    Vector2 p = center(a.search.route[index]);
    if (index + 1 < a.search.route.size()) { Vector2 q = center(a.search.route[index + 1]); p.x += (q.x-p.x)*a.segmentProgress; p.y += (q.y-p.y)*a.segmentProgress; }
    return p;
}
void editCell(App& a, GridCoord p) {
    if (a.tool == Tool::Wall) {
        if (same(p, a.start) || same(p, a.goal)) { a.message = "Endpoints cannot be obstacles."; return; }
        if (!a.grid[p.row][p.col]) { resetRun(a); a.grid[p.row][p.col] = true; a.message = "Obstacle placed."; }
    } else if (a.tool == Tool::Erase) {
        if (a.grid[p.row][p.col]) { resetRun(a); a.grid[p.row][p.col] = false; a.message = "Obstacle erased."; }
    } else {
        if (a.grid[p.row][p.col]) { a.message = "Choose an empty endpoint cell."; return; }
        GridCoord& endpoint = a.tool == Tool::Start ? a.start : a.goal;
        if (!same(endpoint, p)) { resetRun(a); endpoint = p; a.message = "Endpoint updated."; }
    }
}
void text(const std::string& s, int x, int y, int size, Color color) { DrawText(s.c_str(), x, y, size, color); }
void centered(const char* s, Rectangle r, int size, Color color) { DrawText(s, static_cast<int>(r.x + (r.width-MeasureText(s,size))/2), static_cast<int>(r.y + (r.height-size)/2), size, color); }
bool button(Rectangle r, const char* label, bool enabled, bool selected = false) {
    Vector2 m = GetMousePosition(); bool hover = enabled && CheckCollisionPointRec(m, r);
    Color fill = !enabled ? Color{235, 239, 240, 255} : selected ? Blue : hover ? Color{222, 237, 242, 255} : Color{242, 246, 247, 255};
    DrawRectangleRounded(r, 0.18f, 6, fill);
    DrawRectangleRoundedLines(r, 0.18f, 6, selected ? Blue : Line);
    centered(label, r, 17, !enabled ? Color{157, 170, 175, 255} : selected ? WHITE : Ink);
    return hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}
void marker(GridCoord p, const char* label, Color color) {
    const int x = GridX + p.col * CellSize + 2, y = GridY + p.row * CellSize + 2;
    const int width = MeasureText(label, 12) + 6;
    DrawRectangle(x, y, width, 15, color);
    DrawText(label, x + 3, y + 1, 12, WHITE);
}
void drawGrid(const App& a) {
    for (int r = 0; r < Rows; ++r) for (int c = 0; c < Cols; ++c) {
        Rectangle cell{static_cast<float>(GridX+c*CellSize), static_cast<float>(GridY+r*CellSize), static_cast<float>(CellSize), static_cast<float>(CellSize)};
        DrawRectangleRec(cell, a.grid[r][c] ? Wall : Empty);
    }
    for (std::size_t i = 0; i < a.revealed; ++i) { auto p = a.search.expansionOrder[i]; DrawRectangle(GridX+p.col*CellSize+1, GridY+p.row*CellSize+1, CellSize-2, CellSize-2, Explored); }
    GridCoord hover{};
    if (!active(a) && gridCell(GetMousePosition(), hover)) DrawRectangle(GridX+hover.col*CellSize+1, GridY+hover.row*CellSize+1, CellSize-2, CellSize-2, Color{75, 150, 177, 65});
    for (int c=0; c<=Cols; ++c) DrawLine(GridX+c*CellSize, GridY, GridX+c*CellSize, GridY+Rows*CellSize, Line);
    for (int r=0; r<=Rows; ++r) DrawLine(GridX, GridY+r*CellSize, GridX+Cols*CellSize, GridY+r*CellSize, Line);
    for (int c=0; c<Cols; ++c) { std::string s=std::to_string(c); text(s, GridX+c*CellSize+(CellSize-MeasureText(s.c_str(),14))/2, GridY-22, 14, Muted); }
    for (int r=0; r<Rows; ++r) { std::string s=std::to_string(r); text(s, GridX-12-MeasureText(s.c_str(),14), GridY+r*CellSize+11, 14, Muted); }
    if (a.search.status == SearchStatus::Found && a.revealed == a.search.expansionOrder.size()) {
        for (std::size_t i=1; i<a.search.route.size(); ++i) DrawLineEx(center(a.search.route[i-1]), center(a.search.route[i]), 4.0f, Route);
    }
    Vector2 pos = robotPosition(a); DrawCircleV(pos, 10, Ink); DrawCircleV(pos, 7, WHITE); DrawCircleV(pos, 4, Blue);
    Vector2 dir{1,0};
    if (!a.search.route.empty() && a.segment + 1 < a.search.route.size()) { Vector2 next=center(a.search.route[a.segment+1]), prev=center(a.search.route[a.segment]); dir={(next.x-prev.x)/CellSize,(next.y-prev.y)/CellSize}; }
    else if (a.search.route.size() > 1 && a.phase == Phase::Completed) { Vector2 next=center(a.search.route.back()), prev=center(a.search.route[a.search.route.size()-2]); dir={(next.x-prev.x)/CellSize,(next.y-prev.y)/CellSize}; }
    DrawLineEx({pos.x+dir.x*4,pos.y+dir.y*4}, {pos.x+dir.x*13,pos.y+dir.y*13}, 3, Blue);
    if (same(a.start, a.goal)) marker(a.start, "S/G", Green);
    else { marker(a.start, "S", Green); marker(a.goal, "G", Amber); }
}
void drawPanel(App& a) {
    DrawRectangleRounded({PanelX, 96, PanelWidth, 645}, 0.025f, 6, Panel);
    text("CONTROLS", 852, 115, 16, Muted);
    text("Editing tool", 852, 145, 19, Ink);
    const std::array<Tool,4> tools{Tool::Wall,Tool::Erase,Tool::Start,Tool::Goal};
    const std::array<const char*,4> labels{"Place obstacle", "Erase obstacle", "Set start", "Set goal"};
    for (int i=0;i<4;++i) {
        Rectangle r{852.0f+(i%2)*181.0f, 174.0f+(i/2)*42.0f, 173, 34};
        if (button(r, labels[i], !active(a), a.tool==tools[i])) a.tool=tools[i];
    }
    text("Simulation", 852, 270, 19, Ink);
    if (button({852,299,173,36}, "Plan & Run", !active(a))) plan(a);
    if (button({1033,299,173,36}, a.phase==Phase::Paused ? "Resume" : "Pause", active(a))) pauseResume(a);
    if (button({852,343,173,36}, "Reset Run", true)) resetRun(a);
    if (button({1033,343,173,36}, "Clear Walls", true)) { for (auto&& row : a.grid) std::fill(row.begin(), row.end(), false); resetRun(a); a.message="Walls cleared; endpoints preserved."; }
    if (button({852,387,354,36}, "Load Demo", true)) loadDemo(a);
    text("Speed", 852, 440, 19, Ink);
    const std::array<const char*,4> speedLabels{"0.5x","1x","2x","4x"};
    for (int i=0;i<4;++i) if (button({852.0f+i*90.0f,468,84,34},speedLabels[i],true,a.speedIndex==i)) a.speedIndex=i;
    DrawLine(852, 521, 1206, 521, Line);
    text("STATUS", 852, 537, 16, Muted);
    text(phaseName(a.phase), 852, 561, 20, a.phase==Phase::NoRoute ? Color{173,72,56,255} : Ink);
    text(std::string("Tool: ")+toolName(a.tool), 852, 590, 17, Muted);
    text("Explored: "+std::to_string(a.revealed), 852, 614, 17, Muted);
    std::string moves = a.search.status==SearchStatus::Found && !a.search.route.empty() ? std::to_string(a.search.route.size()-1) : "--";
    text("Route: "+moves+" moves     Completed: "+std::to_string(a.segment), 852, 638, 17, Muted);
    text(a.message, 852, 674, 16, Blue);
    text("Enter  Plan   |   Space  Pause   |   R  Reset", 852, 706, 15, Muted);
}
void input(App& a) {
    if (IsKeyPressed(KEY_ENTER) && !active(a)) plan(a);
    if (IsKeyPressed(KEY_SPACE)) pauseResume(a);
    if (IsKeyPressed(KEY_R)) resetRun(a);
    GridCoord p{};
    const bool overGrid = gridCell(GetMousePosition(), p);
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) a.gridDrag = overGrid && !active(a);
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) a.gridDrag = false;
    if (active(a) || !overGrid) return;
    bool dragTool = a.tool==Tool::Wall || a.tool==Tool::Erase;
    if ((dragTool && a.gridDrag && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) || (!dragTool && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))) editCell(a,p);
}
}

int main() {
    InitWindow(WindowWidth, WindowHeight, "Robot Path Planner");
    SetTargetFPS(60);
    App app; loadDemo(app);
    while (!WindowShouldClose()) {
        input(app);
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

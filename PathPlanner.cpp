
#include "PathPlanner.h"
#include <algorithm>
#include <array>
#include <queue>

bool PathPlanner::inBounds(const Grid& grid, GridCoord cell) {
    return cell.row >= 0 && static_cast<std::size_t>(cell.row) < grid.size() &&
           cell.col >= 0 && static_cast<std::size_t>(cell.col) < grid[static_cast<std::size_t>(cell.row)].size();
}

SearchResult PathPlanner::findPath(const Grid& grid, GridCoord start, GridCoord goal) {
    SearchResult result;

    // 1. Validate grid structure
    if (grid.empty() || grid.front().empty()) {
        return result;
    }

    const std::size_t columns = grid.front().size();
    for (const auto& row : grid) {
        if (row.size() != columns) {
            return result;
        }
    }
    
    // 2. Validate start and goal positions
    if (!inBounds(grid, start) || !inBounds(grid, goal) ||
        grid[static_cast<std::size_t>(start.row)][static_cast<std::size_t>(start.col)] ||
        grid[static_cast<std::size_t>(goal.row)][static_cast<std::size_t>(goal.col)]) {
        return result;
    }

    // 3. Initialize search structures
    std::vector<std::vector<bool>> discovered(grid.size(), std::vector<bool>(columns, false));
    std::vector<std::vector<GridCoord>> parent(grid.size(), std::vector<GridCoord>(columns));
    std::queue<GridCoord> pending;

    pending.push(start);
    discovered[static_cast<std::size_t>(start.row)][static_cast<std::size_t>(start.col)] = true;
    
    // A fixed neighbor order makes the chosen shortest path and replay deterministic.
    constexpr std::array<GridCoord, 4> directions{{{-1, 0}, {0, 1}, {1, 0}, {0, -1}}};
    while (!pending.empty()) {
        const GridCoord current = pending.front();
        pending.pop();
        result.expansionOrder.push_back(current);
        if (current == goal) {
            result.status = SearchStatus::Found;
            for (GridCoord cell = goal;; cell = parent[static_cast<std::size_t>(cell.row)][static_cast<std::size_t>(cell.col)]) {
                result.route.push_back(cell);
                if (cell == start) {
                    break;
                }
            }
            std::reverse(result.route.begin(), result.route.end());
            return result;
        }

        for (const GridCoord direction : directions) {
            const GridCoord next{current.row + direction.row, current.col + direction.col};
            if (!inBounds(grid, next)) {
                continue;
            }
            const auto row = static_cast<std::size_t>(next.row);
            const auto col = static_cast<std::size_t>(next.col);
            if (grid[row][col] || discovered[row][col]) {
                continue;
            }
            discovered[row][col] = true;
            parent[row][col] = current;
            pending.push(next);
        }
    }

    result.status = SearchStatus::NoRoute;
    return result;
}

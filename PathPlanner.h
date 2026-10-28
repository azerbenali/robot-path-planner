#pragma once

#include <vector>

struct GridCoord {
    int row = 0;
    int col = 0;
};

inline bool operator==(GridCoord left, GridCoord right) {
    return left.row == right.row && left.col == right.col;
}

inline bool operator!=(GridCoord left, GridCoord right) {
    return !(left == right);
}

// A true cell is an obstacle. All rows in a searchable grid must have equal width.
using Grid = std::vector<std::vector<bool>>;

enum class SearchStatus {
    Found,
    NoRoute,
    InvalidEndpoints
};

struct SearchResult {
    SearchStatus status = SearchStatus::InvalidEndpoints;
    std::vector<GridCoord> route;
    std::vector<GridCoord> expansionOrder;
};

class PathPlanner {
public:
    static bool inBounds(const Grid& grid, GridCoord cell);
    static SearchResult findPath(const Grid& grid, GridCoord start, GridCoord goal);
};

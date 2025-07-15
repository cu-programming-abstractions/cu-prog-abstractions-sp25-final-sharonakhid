/**
 * Dungeon Pathfinder - BFS Solver
 * 
 * This file implements BFS pathfinding algorithms for dungeon navigation.
 */

#include "solver.h"
#include "cell.h"
#include <vector>
#include <algorithm>
#include <string>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

using namespace std;

/**
 * Helper function: Find a specific character in the dungeon
 * Returns Cell(-1, -1) if not found
 */
Cell findPosition(const vector<string>& dungeon, char target) {
    for (size_t row = 0; row < dungeon.size(); row++) {
        for (size_t col = 0; col < dungeon[row].size(); col++) {
            if (dungeon[row][col] == target) {
                return Cell(static_cast<int>(row), static_cast<int>(col));  // convert size_t to int
            }
        }
    }
    return Cell(-1, -1);  // Not found
}

/**
 * Helper function: Check if a position is passable for basic BFS
 * (not a wall or door, and within bounds)
 */
bool isPassable(const vector<string>& dungeon, int row, int col) {
    if (row < 0 || row >= static_cast<int>(dungeon.size()) ||
        col < 0 || col >= static_cast<int>(dungeon[0].size())) {
        return false;  // Out of bounds
    }
    
    char cell = dungeon[row][col];
    
    // Walls are impassable
    if (cell == '#') return false;
    
    // Doors are impassable for basic BFS (key-door BFS handles doors separately)
    // Note: 'E' (exit) is not a door, so exclude it from door check
    if (cell >= 'A' && cell <= 'F' && cell != 'E') return false;
    
    // Everything else (spaces, S, E, keys a-f) is passable
    return true;
}

/**
 * Helper function: Check if we can pass through a door
 * Door 'A' requires key 'a', door 'B' requires key 'b', etc.
 */
bool canPassDoor(char door, int keyMask) {
    if (door < 'A' || door > 'F') return true;  // Not a door
    
    char requiredKey = door - 'A' + 'a';  // Convert 'A' to 'a', 'B' to 'b', etc.
    int keyBit = requiredKey - 'a';       // Convert to bit position (0-5)
    
    return (keyMask >> keyBit) & 1;       // Check if that bit is set
}

/**
 * Helper function: Collect a key by setting the appropriate bit
 */
int collectKey(char key, int keyMask) {
    if (key < 'a' || key > 'f') return keyMask;  // Not a key
    
    int keyBit = key - 'a';              // Convert to bit position (0-5)
    return keyMask | (1 << keyBit);      // Set that bit
}

/**
 * Helper function: Reconstruct path from parent pointers
 */
vector<Cell> reconstructPath(const unordered_map<Cell, Cell, CellHash>& parents, 
                            const Cell& start, const Cell& goal) {
    vector<Cell> path;
    Cell current = goal;
    
    // Follow parent pointers back to start
    while (!(current.r == start.r && current.c == start.c)) {
        path.push_back(current);
        auto it = parents.find(current);
        if (it == parents.end()) {
            return vector<Cell>(); // Path reconstruction failed
        }
        current = it->second;
    }
    path.push_back(start);
    
    reverse(path.begin(), path.end());
    return path;
}

/**
 * Helper function: Check if a position is within bounds and not a wall
 * (used by key-door BFS which handles doors separately)
 */
bool isValidPosition(const vector<string>& dungeon, int row, int col) {
    if (row < 0 || row >= static_cast<int>(dungeon.size()) ||
        col < 0 || col >= static_cast<int>(dungeon[0].size())) {
        return false;  // Out of bounds
    }
    
    // Only walls are impassable for key-door BFS (doors handled separately)
    return dungeon[row][col] != '#';
}

/**
 * Helper function: Get all valid neighboring cells for basic BFS
 */
vector<Cell> getNeighbors(const vector<string>& dungeon, const Cell& current) {
    vector<Cell> neighbors;
    
    for (int i = 0; i < NUM_DIRECTIONS; i++) {
        int newRow = current.r + DIRECTIONS[i][0];
        int newCol = current.c + DIRECTIONS[i][1];
        
        if (isPassable(dungeon, newRow, newCol)) {
            neighbors.push_back(Cell(newRow, newCol));
        }
    }
    
    return neighbors;
}

std::vector<Cell> bfsPath(const std::vector<std::string>& dungeon) {
    Cell start = findPosition(dungeon, 'S');
    Cell exit = findPosition(dungeon, 'E');

    if (start.r == -1 || exit.r == -1) {
        return vector<Cell>();  // Invalid dungeon
    }

    queue<Cell> bfsQueue;
    unordered_set<Cell, CellHash> visited;
    unordered_map<Cell, Cell, CellHash> parents;

    bfsQueue.push(start);
    visited.insert(start);

    cout << "Starting BFS from (" << start.r << "," << start.c << ") to ("
         << exit.r << "," << exit.c << ")" << endl;

    while (!bfsQueue.empty()) {
        Cell current = bfsQueue.front();
        bfsQueue.pop();

        if (current == exit) {
            return reconstructPath(parents, start, exit);
        }

        vector<Cell> neighbors = getNeighbors(dungeon, current);
        for (const Cell& neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                bfsQueue.push(neighbor);
                visited.insert(neighbor);
                parents[neighbor] = current;
            }
        }
    }

    return vector<Cell>();  // No path found
}

/**
 * State structure for key-door BFS that includes position and collected keys.
 * 
 * CONCEPT: In basic BFS, state = (row, col). In key-door BFS, state = (row, col, keys).
 * The same position with different keys represents different states with different possibilities.
 * 
 * See BITMASK_BFS_GUIDE.md for detailed explanation of state augmentation concepts.
 */
struct KeyState {
    int row, col, keyMask;

    KeyState() : row(0), col(0), keyMask(0) {}  // ✅ default constructor
    KeyState(int r, int c, int k) : row(r), col(c), keyMask(k) {}

    bool operator==(const KeyState& other) const {
        return row == other.row && col == other.col && keyMask == other.keyMask;
    }
};

struct KeyStateHash {
    size_t operator()(const KeyState& k) const {
        return std::hash<int>()(k.row) ^ (std::hash<int>()(k.col) << 1) ^ (std::hash<int>()(k.keyMask) << 2);
    }
};



/**
 * BITMASK OVERVIEW:
 * A bitmask efficiently stores which keys we have using a single integer.
 * Each bit represents one key: bit 0 = key 'a', bit 1 = key 'b', etc.
 * 
 * Examples: keyMask=0 (no keys), keyMask=1 (key 'a'), keyMask=3 (keys 'a' and 'b')
 * 
 * For detailed explanation and practice exercises, see BITMASK_BFS_GUIDE.md
 */

/**
 * Helper function to convert a keyMask to a readable string for debugging.
 */
string keyMaskToString(int keyMask) {
    string result = "Keys: ";
    bool hasAny = false;
    for (int i = 0; i < 6; i++) {  // Check keys 'a' through 'f'
        if ((keyMask >> i) & 1) {  // If bit i is set
            result += char('a' + i);
            result += " ";
            hasAny = true;
        }
    }
    if (!hasAny) result += "none";
    return result;
}

/**
 * Helper function to reconstruct path from KeyState parents.
 */
vector<Cell> reconstructKeyPath(const unordered_map<KeyState, KeyState, KeyStateHash>& parents,
                               const KeyState& start, const KeyState& goal) {
    vector<Cell> path;
    KeyState current = goal;
    
    // Follow parent pointers back to start
    while (!(current.row == start.row && current.col == start.col && current.keyMask == start.keyMask)) {
        path.push_back(Cell(current.row, current.col));
        auto it = parents.find(current);
        if (it == parents.end()) {
            return vector<Cell>(); // Path reconstruction failed
        }
        current = it->second;
    }
    path.push_back(Cell(start.row, start.col));
    
    reverse(path.begin(), path.end());
    return path;
}

std::vector<Cell> bfsPathKeys(const std::vector<std::string>& dungeon) {
    Cell start = findPosition(dungeon, 'S');
    Cell exit = findPosition(dungeon, 'E');

    if (start.r == -1 || exit.r == -1) {
        return vector<Cell>();
    }

    queue<KeyState> bfsQueue;
    unordered_set<KeyState, KeyStateHash> visited;
    unordered_map<KeyState, KeyState, KeyStateHash> parents;

    KeyState startState(start.r, start.c, 0);
    bfsQueue.push(startState);
    visited.insert(startState);

    cout << "Starting key-door BFS from (" << start.r << "," << start.c << ")" << endl;

    while (!bfsQueue.empty()) {
        KeyState current = bfsQueue.front();
        bfsQueue.pop();

        if (current.row == exit.r && current.col == exit.c) {
            cout << "Exit found!" << endl;
            return reconstructKeyPath(parents, startState, current);
        }

        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            int newRow = current.row + DIRECTIONS[i][0];
            int newCol = current.col + DIRECTIONS[i][1];

            if (!isValidPosition(dungeon, newRow, newCol)) {
                continue;
            }

            char cellChar = dungeon[newRow][newCol];

            // Implement door checking logic
            if (cellChar >= 'A' && cellChar <= 'F' && !canPassDoor(cellChar, current.keyMask)) {
                continue;  // Skip this neighbor if we can't pass the door
            }

            // Implement key collection logic
            int newKeyMask = current.keyMask;
            if (cellChar >= 'a' && cellChar <= 'f') {
                newKeyMask = collectKey(cellChar, newKeyMask);
            }

            // Create new state with updated key information
            KeyState newState(newRow, newCol, newKeyMask);

            if (visited.find(newState) == visited.end()) {
                bfsQueue.push(newState);
                visited.insert(newState);
                parents[newState] = current;
            }
        }
    }

    return vector<Cell>();  // No path found
}

vector<Cell> bfsPathKeys(const vector<string>& dungeon, KeyState start, KeyState exit) {
    static const int DIRECTIONS[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
    unordered_set<KeyState, KeyStateHash> visited;
    unordered_map<KeyState, KeyState, KeyStateHash> parents;
    queue<KeyState> bfsQueue;

    visited.insert(start);
    bfsQueue.push(start);

    while (!bfsQueue.empty()) {
        KeyState current = bfsQueue.front();
        bfsQueue.pop();

        if (current.row == exit.row && current.col == exit.col) {
            return reconstructKeyPath(parents, start, current);  // ✅ returns vector<Cell>
        }

        for (int i = 0; i < 4; i++) {
            int newRow = current.row + DIRECTIONS[i][0];
            int newCol = current.col + DIRECTIONS[i][1];

            if (newRow < 0 || newRow >= (int)dungeon.size() || newCol < 0 || newCol >= (int)dungeon[0].size())
                continue;

            char cellChar = dungeon[newRow][newCol];

            if (cellChar == '#')
                continue;

            if (cellChar >= 'A' && cellChar <= 'F' && !canPassDoor(cellChar, current.keyMask))
                continue;

            int newKeyMask = current.keyMask;
            if (cellChar >= 'a' && cellChar <= 'f') {
                newKeyMask = collectKey(cellChar, newKeyMask);
            }

            KeyState newState(newRow, newCol, newKeyMask);
            if (visited.find(newState) == visited.end()) {
                visited.insert(newState);
                parents[newState] = current;
                bfsQueue.push(newState);
            }
        }
    }

    return {};  // No path found
}

#ifdef IMPLEMENT_OPTIONAL_FUNCTIONS
#include <vector>
#include <queue>
#include <unordered_set>

int countReachableKeys(const std::vector<std::string>& dungeon) {
    Cell start = findPosition(dungeon, 'S');
    if (start.r == -1) return 0;

    std::queue<Cell> queue;
    std::unordered_set<Cell, CellHash> visited;
    int keyMask = 0;

    // Initialize BFS structures
    queue.push(start);
    visited.insert(start);

    // BFS loop
    while (!queue.empty()) {
        Cell current = queue.front();
        queue.pop();

        // Check if current cell is a key
        char cellValue = dungeon[current.r][current.c];
        if (cellValue >= 'a' && cellValue <= 'f') {
            // Collect the key
            keyMask |= (1 << (cellValue - 'a'));
        }

        // Explore neighbors
        for (const auto& neighbor : getNeighbors(current, dungeon)) {
            if (visited.find(neighbor) == visited.end() && isPassable(neighbor, dungeon)) {
                visited.insert(neighbor);
                queue.push(neighbor);
            }
        }
    }

    // Count the number of set bits in keyMask
    int count = 0;
    for (int i = 0; i < 6; ++i) {
        if (keyMask & (1 << i)) {
            count++;
        }
    }

    return count;  // Return the actual count of collected keys
}

#else
/**
 * Stub implementation when optional functions are disabled.
 * This prevents compilation errors when the function is not implemented.
 */
int countReachableKeys(const std::vector<std::string>& dungeon) {
    (void)dungeon; // to silence unused parameter warning
    return 0;
}
#endif

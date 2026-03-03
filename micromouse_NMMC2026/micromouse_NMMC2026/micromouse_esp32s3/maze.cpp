// ═══════════════════════════════════════════════════════════════════════════════
//  maze.cpp — 16×16 Maze + Flood Fill Implementation
//
//  Flood fill uses BFS (breadth-first search) from goal cells.
//  Queue implemented as simple ring buffer — no dynamic allocation.
// ═══════════════════════════════════════════════════════════════════════════════
#include "maze.h"
#include <string.h>

// ── BFS Queue (static, no malloc) ────────────────────────────────────────────
#define QUEUE_SIZE  MAZE_CELLS   // worst case: every cell in queue
typedef struct {
    uint8_t x[QUEUE_SIZE];
    uint8_t y[QUEUE_SIZE];
    uint16_t head, tail, count;
} BfsQueue;

static BfsQueue s_q;

static inline void q_clear(void)           { s_q.head = s_q.tail = s_q.count = 0; }
static inline bool q_empty(void)            { return s_q.count == 0; }
static inline void q_push(uint8_t x, uint8_t y) {
    if (s_q.count >= QUEUE_SIZE) return;
    s_q.x[s_q.tail] = x;
    s_q.y[s_q.tail] = y;
    s_q.tail = (s_q.tail + 1) % QUEUE_SIZE;
    s_q.count++;
}
static inline void q_pop(uint8_t* x, uint8_t* y) {
    *x = s_q.x[s_q.head];
    *y = s_q.y[s_q.head];
    s_q.head = (s_q.head + 1) % QUEUE_SIZE;
    s_q.count--;
}

// ── Delta tables ─────────────────────────────────────────────────────────────
//                          N   E   S   W
static const int8_t dx[] = { 0,  1,  0, -1};
static const int8_t dy[] = { 1,  0, -1,  0};

// ═══════════════════════════════════════════════════════════════════════════════
// Init
// ═══════════════════════════════════════════════════════════════════════════════
void maze_init(Maze* m) {
    memset(m->walls, 0, sizeof(m->walls));
    memset(m->flood, 0xFF, sizeof(m->flood));   // all FLOOD_MAX

    // Set outer boundary walls
    for (uint8_t i = 0; i < MAZE_W; i++) {
        m->walls[0][i]          |= WALL_S;     // bottom row: south wall
        m->walls[MAZE_H-1][i]   |= WALL_N;     // top row: north wall
    }
    for (uint8_t j = 0; j < MAZE_H; j++) {
        m->walls[j][0]          |= WALL_W;     // left col: west wall
        m->walls[j][MAZE_W-1]   |= WALL_E;     // right col: east wall
    }

    // Default goal: center 2×2 (cells 7,7 to 8,8)
    m->goal_x_min = 7;  m->goal_y_min = 7;
    m->goal_x_max = 8;  m->goal_y_max = 8;

    // ★ Start cell (0,0): always has S+W+E walls, N exit
    //   (matches mazerunner-core: set_wall_state(START, EAST, WALL) + NORTH=EXIT)
    //   S and W are already set by boundary loop above.
    maze_set_wall(m, 0, 0, DIR_E);   // east wall confirmed
    // North is left as 0 (no wall) = exit — matches reference
}

// ═══════════════════════════════════════════════════════════════════════════════
// Wall access (auto-set neighbor's reciprocal wall)
// ═══════════════════════════════════════════════════════════════════════════════
void maze_set_wall(Maze* m, uint8_t x, uint8_t y, uint8_t dir) {
    if (x >= MAZE_W || y >= MAZE_H || dir >= DIR_COUNT) return;
    m->walls[y][x] |= (1 << dir);

    // Set neighbor's reciprocal wall
    uint8_t nx, ny;
    if (maze_step(x, y, dir, &nx, &ny)) {
        m->walls[ny][nx] |= (1 << dir_opposite(dir));
    }
}

void maze_clear_wall(Maze* m, uint8_t x, uint8_t y, uint8_t dir) {
    if (x >= MAZE_W || y >= MAZE_H || dir >= DIR_COUNT) return;
    m->walls[y][x] &= ~(1 << dir);

    uint8_t nx, ny;
    if (maze_step(x, y, dir, &nx, &ny)) {
        m->walls[ny][nx] &= ~(1 << dir_opposite(dir));
    }
}

bool maze_has_wall(const Maze* m, uint8_t x, uint8_t y, uint8_t dir) {
    if (x >= MAZE_W || y >= MAZE_H) return true;   // out of bounds = wall
    return (m->walls[y][x] & (1 << dir)) != 0;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Visited
// ═══════════════════════════════════════════════════════════════════════════════
void maze_set_visited(Maze* m, uint8_t x, uint8_t y) {
    if (x < MAZE_W && y < MAZE_H) m->walls[y][x] |= CELL_VISITED;
}

bool maze_is_visited(const Maze* m, uint8_t x, uint8_t y) {
    if (x >= MAZE_W || y >= MAZE_H) return false;
    return (m->walls[y][x] & CELL_VISITED) != 0;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Flood fill — BFS from goal cells
// ═══════════════════════════════════════════════════════════════════════════════
void maze_flood(Maze* m) {
    // Initialize all cells to FLOOD_MAX
    for (uint8_t y = 0; y < MAZE_H; y++)
        for (uint8_t x = 0; x < MAZE_W; x++)
            m->flood[y][x] = FLOOD_MAX;

    // Seed: goal cells = 0
    q_clear();
    for (uint8_t y = m->goal_y_min; y <= m->goal_y_max; y++) {
        for (uint8_t x = m->goal_x_min; x <= m->goal_x_max; x++) {
            m->flood[y][x] = 0;
            q_push(x, y);
        }
    }

    // BFS expansion
    while (!q_empty()) {
        uint8_t cx, cy;
        q_pop(&cx, &cy);
        uint16_t cost = m->flood[cy][cx] + 1;

        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            // Skip if wall blocks
            if (maze_has_wall(m, cx, cy, d)) continue;

            uint8_t nx, ny;
            if (!maze_step(cx, cy, d, &nx, &ny)) continue;

            // Update if shorter path found
            if (cost < m->flood[ny][nx]) {
                m->flood[ny][nx] = cost;
                q_push(nx, ny);
            }
        }
    }
}

void maze_flood_target(Maze* m, uint8_t tx, uint8_t ty) {
    // Save goal, flood toward single cell, restore goal
    uint8_t ox0 = m->goal_x_min, oy0 = m->goal_y_min;
    uint8_t ox1 = m->goal_x_max, oy1 = m->goal_y_max;
    m->goal_x_min = m->goal_x_max = tx;
    m->goal_y_min = m->goal_y_max = ty;
    maze_flood(m);
    m->goal_x_min = ox0; m->goal_y_min = oy0;
    m->goal_x_max = ox1; m->goal_y_max = oy1;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Navigation
// ═══════════════════════════════════════════════════════════════════════════════

int8_t maze_best_dir(const Maze* m, uint8_t x, uint8_t y) {
    uint16_t best_cost = FLOOD_MAX;
    int8_t   best_dir  = -1;

    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        if (maze_has_wall(m, x, y, d)) continue;

        uint8_t nx, ny;
        if (!maze_step(x, y, d, &nx, &ny)) continue;

        if (m->flood[ny][nx] < best_cost) {
            best_cost = m->flood[ny][nx];
            best_dir  = (int8_t)d;
        }
    }
    return best_dir;
}

int8_t maze_turn_needed(uint8_t current_heading, uint8_t desired_dir) {
    // Returns: 0=straight, 1=right(CW), 2=u-turn, 3=left(CCW)
    return (int8_t)((desired_dir - current_heading + 4) & 3);
}

bool maze_step(uint8_t x, uint8_t y, uint8_t dir, uint8_t* nx, uint8_t* ny) {
    int8_t nxi = (int8_t)x + dx[dir];
    int8_t nyi = (int8_t)y + dy[dir];
    if (nxi < 0 || nxi >= MAZE_W || nyi < 0 || nyi >= MAZE_H) return false;
    *nx = (uint8_t)nxi;
    *ny = (uint8_t)nyi;
    return true;
}

bool maze_is_goal(const Maze* m, uint8_t x, uint8_t y) {
    return x >= m->goal_x_min && x <= m->goal_x_max &&
           y >= m->goal_y_min && y <= m->goal_y_max;
}

const char* dir_name(uint8_t d) {
    static const char* names[] = {"N","E","S","W"};
    return (d < 4) ? names[d] : "?";
}

// ═══════════════════════════════════════════════════════════════════════════════
// Serialization (for BLE → GUI maze display)
// ═══════════════════════════════════════════════════════════════════════════════

void maze_pack_walls(const Maze* m, uint8_t* buf128) {
    // Pack 2 cells per byte (low nibble = even cell, high nibble = odd cell)
    // Row-major: y=0..15, x=0..15
    memset(buf128, 0, 128);
    for (uint8_t y = 0; y < MAZE_H; y++) {
        for (uint8_t x = 0; x < MAZE_W; x++) {
            uint16_t idx = y * MAZE_W + x;
            uint8_t val = m->walls[y][x] & CELL_WALLS_MASK;
            if (idx & 1) {
                buf128[idx >> 1] |= (val << 4);
            } else {
                buf128[idx >> 1] |= val;
            }
        }
    }
}

void maze_pack_flood(const Maze* m, uint8_t* buf256) {
    for (uint8_t y = 0; y < MAZE_H; y++) {
        for (uint8_t x = 0; x < MAZE_W; x++) {
            uint16_t v = m->flood[y][x];
            buf256[y * MAZE_W + x] = (v > 255) ? 255 : (uint8_t)v;
        }
    }
}

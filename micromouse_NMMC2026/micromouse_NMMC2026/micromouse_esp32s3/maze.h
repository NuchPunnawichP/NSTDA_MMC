// ═══════════════════════════════════════════════════════════════════════════════
//  maze.h — 16×16 Maze Data Structure + Flood Fill
//
//  Memory: compact bit-packed walls (1 byte per cell = 512 bytes)
//         + flood values (2 bytes per cell = 512 bytes)
//         Total: ~1 KB for entire maze
//
//  Wall storage:  Each cell stores its OWN walls as 4 bits: [N W S E]
//                 When you set a wall, the neighbor's wall is auto-set.
//
//  Flood fill:    BFS from goal → every cell gets cost (distance to goal)
//                 The robot always moves to the neighbor with lowest cost.
//
//  Coordinate system:
//    (0,0) = bottom-left (start cell)
//    X = column (East+), Y = row (North+)
//    Heading: 0=N, 1=E, 2=S, 3=W
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── Constants ────────────────────────────────────────────────────────────────
#define MAZE_W      16
#define MAZE_H      16
#define MAZE_CELLS  (MAZE_W * MAZE_H)       // 256
#define FLOOD_MAX   0xFFFF                   // unreachable sentinel

// ── Directions ───────────────────────────────────────────────────────────────
// Heading index: 0=North, 1=East, 2=South, 3=West
#define DIR_N   0
#define DIR_E   1
#define DIR_S   2
#define DIR_W   3
#define DIR_COUNT 4

// ── Wall bits (per cell, bit mask) ───────────────────────────────────────────
#define WALL_N  (1 << DIR_N)    // 0x01
#define WALL_E  (1 << DIR_E)    // 0x02
#define WALL_S  (1 << DIR_S)    // 0x04
#define WALL_W  (1 << DIR_W)    // 0x08

// ── "Visited" flag packed into upper nibble ──────────────────────────────────
#define CELL_VISITED    0x10
#define CELL_WALLS_MASK 0x0F

// ── Maze structure ───────────────────────────────────────────────────────────
typedef struct {
    // walls[y][x]: lower 4 bits = walls (NSEW), bit4 = visited
    uint8_t  walls[MAZE_H][MAZE_W];

    // flood[y][x]: distance to goal (0 = goal cell, FLOOD_MAX = unreachable)
    uint16_t flood[MAZE_H][MAZE_W];

    // Goal rectangle (configurable, default = center 2×2)
    uint8_t goal_x_min, goal_y_min, goal_x_max, goal_y_max;
} Maze;

// ═══════════════════════════════════════════════════════════════════════════════
// API
// ═══════════════════════════════════════════════════════════════════════════════

// ── Init / Reset ─────────────────────────────────────────────────────────────
//  Sets outer boundary walls, clears interior, sets goal.
void maze_init(Maze* m);

// ── Wall access ──────────────────────────────────────────────────────────────
//  set_wall: sets wall for cell (x,y) in direction dir, AND the neighbor's wall
void maze_set_wall(Maze* m, uint8_t x, uint8_t y, uint8_t dir);
void maze_clear_wall(Maze* m, uint8_t x, uint8_t y, uint8_t dir);
bool maze_has_wall(const Maze* m, uint8_t x, uint8_t y, uint8_t dir);

// ── Visited ──────────────────────────────────────────────────────────────────
void maze_set_visited(Maze* m, uint8_t x, uint8_t y);
bool maze_is_visited(const Maze* m, uint8_t x, uint8_t y);

// ── Flood fill ───────────────────────────────────────────────────────────────
//  BFS from goal cells. Respects known walls.
//  After calling, flood[y][x] = steps to reach nearest goal cell.
void maze_flood(Maze* m);

//  Flood variant: flood toward a single target cell (e.g. return to start)
void maze_flood_target(Maze* m, uint8_t tx, uint8_t ty);

// ── Navigation helpers ───────────────────────────────────────────────────────

//  Given robot at (x,y) with heading h, which direction should it go?
//  Returns direction (DIR_N/E/S/W) toward the neighbor with lowest flood value
//  that doesn't have a wall blocking.  Returns -1 if stuck.
int8_t maze_best_dir(const Maze* m, uint8_t x, uint8_t y);

//  Relative turn needed: given current heading and desired direction
//  Returns: 0=straight, 1=turn right, 2=turn 180, 3=turn left (-1)
int8_t maze_turn_needed(uint8_t current_heading, uint8_t desired_dir);

//  Step in direction: compute (nx, ny) from (x, y) + dir
//  Returns false if out of bounds
bool maze_step(uint8_t x, uint8_t y, uint8_t dir, uint8_t* nx, uint8_t* ny);

//  Is (x,y) a goal cell?
bool maze_is_goal(const Maze* m, uint8_t x, uint8_t y);

//  Opposite direction (N↔S, E↔W)
static inline uint8_t dir_opposite(uint8_t d) { return (d + 2) & 3; }

//  Turn direction string (for debug)
const char* dir_name(uint8_t d);

// ── Serialization (for BLE transfer of maze to GUI) ──────────────────────────
//  Pack maze walls into 128-byte buffer (4 bits per cell, row-major)
void maze_pack_walls(const Maze* m, uint8_t* buf128);

//  Pack flood values into compact format (optional, for debug)
//  Uses 1 byte per cell (clamped to 255), row-major = 256 bytes
void maze_pack_flood(const Maze* m, uint8_t* buf256);

#ifdef __cplusplus
}
#endif

#endif // MAZE_H

#ifndef FUNDAMENTALPATH_H
#define FUNDAMENTALPATH_H

#include <stdbool.h>
#include "../direction.h"

struct MapPoint; // ✅ Forward declaration of MapPoint

typedef struct FundamentalPath
{
    int id;
    struct MapPoint *start;
    struct MapPoint *end;
    int distance;
    Direction direction;
    bool dead_end; // Flag to mark if this path leads to a dead end
} FundamentalPath;

// Function prototypes
void initialize_fundamental_path(FundamentalPath *fp, struct MapPoint *start, int distance, Direction direction);
FundamentalPath *initialize_fundamental_paths(bool UltraSonicDetection[3]);

// ✅ Now it's safe to use MapPoint in function signatures
void update_latest_fundamental_path(struct MapPoint *current, struct MapPoint *former);

/**
 * @brief Marks a fundamental path as a dead end.
 *
 * @param path Pointer to the FundamentalPath to mark as a dead end.
 */
void add_dead_end_flag_to_fundamental_path(FundamentalPath *path);

#endif // FUNDAMENTALPATH_H

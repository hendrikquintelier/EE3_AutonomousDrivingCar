#ifndef FUNDAMENTALPATH_H
#define FUNDAMENTALPATH_H

#include <stdbool.h>
#include "../direction.h"

struct MapPoint;  // ✅ Forward declaration of MapPoint


typedef struct FundamentalPath {
    int id;
    struct MapPoint *start;
    struct MapPoint *end;
    int distance;
    Direction direction;
} FundamentalPath;

// Function prototypes
void initialize_fundamental_path(FundamentalPath *fp, struct MapPoint *start, int distance);
FundamentalPath* initialize_fundamental_paths(bool UltraSonicDetection[3]);

// ✅ Now it's safe to use MapPoint in function signatures
void update_latest_fundamental_path(struct MapPoint* current, struct MapPoint* former);

#endif // FUNDAMENTALPATH_H

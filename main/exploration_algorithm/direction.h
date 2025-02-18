#ifndef DIRECTION_H
#define DIRECTION_H



// Forward declaration (this tells the compiler that MapPoint is a struct)
struct MapPoint;
typedef struct MapPoint MapPoint;

typedef enum {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
    INVALID_DIRECTION = -1  // Define an invalid direction
} Direction;

#include "algorithm_structs_PUBLIC/FundamentalPath.h"  // Include this to ensure Direction is defined

// Function prototypes
Direction determine_direction(MapPoint *start, MapPoint *end);
Direction opposite_direction(Direction dir);
Direction turn_left(Direction dir);
Direction turn_right(Direction dir);
const char* direction_to_string(Direction dir);
char direction_to_symbol(Direction dir);

#endif // DIRECTION_H

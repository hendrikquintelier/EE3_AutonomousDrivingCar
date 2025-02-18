#ifndef PATH_H
#define PATH_H

#include "MapPoint.h"
#include "FundamentalPath.h"

typedef struct Path {
    MapPoint *start;
    MapPoint *end;
    FundamentalPath **route;
    int totalDistance;
} Path;

void initialize_path(Path *path, MapPoint *start, MapPoint *end);
void printPathResult(const Path *path);

#endif // PATH_H

#ifndef GLOBALS_H
#define GLOBALS_H

#include "algorithm_structs_PUBLIC/MapPoint.h"
#include "algorithm_structs_PUBLIC/FundamentalPath.h"

extern MapPoint **map_points_tbd;
extern MapPoint **map_points_all;
extern FundamentalPath **all_fundamental_paths;

extern int num_map_points_tbd, capacity_map_points_tbd;
extern int num_map_points_all, capacity_map_points_all;
extern int num_all_fundamental_paths, capacity_all_fundamental_paths;

extern Location start;
extern Direction start_orientation;

// Define the Car struct
typedef struct {
    Location current_location;
    char current_orientation;
} Car;

// Global Car instance
extern Car current_car;

// Global ultrasonic sensor readings (0: forward, 1: left, 2: right)
extern bool ultrasonic_sensors[3];

void initialize_globals();
void free_globals();
void check_mappoints_tbd();
void add_fundamental_path(FundamentalPath *path);
#endif // GLOBALS_H

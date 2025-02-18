#include <stdlib.h>
#include <stdio.h>
#include "globals.h"
#include "direction.h"
#include "track_files_PRIVATE/track_navigation.h"

// Dynamic global arrays
MapPoint **map_points_tbd = NULL;
MapPoint **map_points_all = NULL;
FundamentalPath **all_fundamental_paths = NULL;


Location start = {0,0};
Direction start_orientation = SOUTH;




// Define global car
Car current_car = {{2, 1}, EAST};

// Define global ultrasonic sensors
bool ultrasonic_sensors[3] = {true, true, true};  // {forward, left, right}

// Sizes and capacities
int num_map_points_tbd = 0, capacity_map_points_tbd = 20;
int num_map_points_all = 0, capacity_map_points_all = 80;
int num_all_fundamental_paths = 0, capacity_all_fundamental_paths = 160;

void initialize_globals() {
    map_points_tbd = malloc(capacity_map_points_tbd * sizeof(MapPoint));
    map_points_all = malloc(capacity_map_points_all * sizeof(MapPoint));
    all_fundamental_paths = malloc(capacity_all_fundamental_paths * sizeof(FundamentalPath));

    if (!map_points_tbd || !map_points_all || !all_fundamental_paths) {
        perror("Failed to allocate global arrays");
        exit(EXIT_FAILURE);
    }
}

void free_globals() {
    free(map_points_tbd);
    free(map_points_all);
    free(all_fundamental_paths);
}

void check_mappoints_tbd() {
    for (int i = num_map_points_tbd - 1; i >= 0; i--) {
        MapPoint *map_point_tbd = map_points_tbd[i];
        if (!mp_has_unexplored_paths(map_point_tbd)) {
            // Shift elements to the left to fill the gap
            for (int j = i; j < num_map_points_tbd - 1; j++) {
                map_points_tbd[j] = map_points_tbd[j + 1];
            }
            num_map_points_tbd--;

        }
    }
}

// Function to add a FundamentalPath to the global list
void add_fundamental_path(FundamentalPath *path) {
    // Reallocate memory to accommodate the new path
    FundamentalPath **temp = realloc(all_fundamental_paths, (num_all_fundamental_paths + 1) * sizeof(FundamentalPath *));
    if (!temp) {
        perror("Failed to reallocate memory for all_fundamental_paths");
        exit(EXIT_FAILURE);
    }

    // Update the global array and counter
    all_fundamental_paths = temp;
    all_fundamental_paths[num_all_fundamental_paths++] = path;
}





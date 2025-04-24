#include <stdlib.h>
#include <stdio.h>
#include "globals.h"
#include "direction.h"

// Dynamic global arrays
MapPoint **map_points_tbd = NULL;
MapPoint **map_points_all = NULL;
FundamentalPath **all_fundamental_paths = NULL;

// Ultrasonic distance variables
float front_distance_ultrasonic = 0.0;
float left_distance_ultrasonic = 0.0;
float right_distance_ultrasonic = 0.0;

Location start = {0, 0};
Direction start_orientation = NORTH;

// Define global car
Car current_car = {{0, 0}, NORTH};

// Define global ultrasonic sensors
bool ultrasonic_sensors[3] = {false, false, false}; // {forward, left, right}

// Sizes and capacities
int num_map_points_tbd = 0, capacity_map_points_tbd = 20;                // Initial capacity of 20
int num_map_points_all = 0, capacity_map_points_all = 80;                // Initial capacity of 80
int num_all_fundamental_paths = 0, capacity_all_fundamental_paths = 160; // Initial capacity of 160

void initialize_globals()
{
    front_distance_ultrasonic = 0.0;
    left_distance_ultrasonic = 0.0;
    right_distance_ultrasonic = 0.0;

    // Allocate memory for arrays of pointers
    map_points_tbd = (MapPoint **)malloc(capacity_map_points_tbd * sizeof(MapPoint *));
    map_points_all = (MapPoint **)malloc(capacity_map_points_all * sizeof(MapPoint *));
    all_fundamental_paths = (FundamentalPath **)malloc(capacity_all_fundamental_paths * sizeof(FundamentalPath *));

    if (!map_points_tbd || !map_points_all || !all_fundamental_paths)
    {
        perror("Failed to allocate global arrays");
        return; // Return instead of exit to allow error handling
    }

    // Initialize all pointers to NULL
    for (int i = 0; i < capacity_map_points_tbd; i++)
    {
        map_points_tbd[i] = NULL;
    }
    for (int i = 0; i < capacity_map_points_all; i++)
    {
        map_points_all[i] = NULL;
    }
    for (int i = 0; i < capacity_all_fundamental_paths; i++)
    {
        all_fundamental_paths[i] = NULL;
    }
}

void free_globals()
{
    free(map_points_tbd);
    free(map_points_all);
    free(all_fundamental_paths);
}

void check_mappoints_tbd()
{
    for (int i = num_map_points_tbd - 1; i >= 0; i--)
    {
        MapPoint *map_point_tbd = map_points_tbd[i];
        if (!mp_has_unexplored_paths(map_point_tbd))
        {
            // Shift elements to the left to fill the gap
            for (int j = i; j < num_map_points_tbd - 1; j++)
            {
                map_points_tbd[j] = map_points_tbd[j + 1];
            }
            num_map_points_tbd--;
        }
    }
}

// Function to add a FundamentalPath to the global list
void add_fundamental_path(FundamentalPath *path)
{
    // Reallocate memory to accommodate the new path
    FundamentalPath **temp = realloc(all_fundamental_paths, (num_all_fundamental_paths + 1) * sizeof(FundamentalPath *));
    if (!temp)
    {
        perror("Failed to reallocate memory for all_fundamental_paths");
        exit(EXIT_FAILURE);
    }

    // Update the global array and counter
    all_fundamental_paths = temp;
    all_fundamental_paths[num_all_fundamental_paths++] = path;
}

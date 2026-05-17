#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../chassis_control/Inc/path_plan.h"

/*
 * PC-side test harness for path_plan.
 *
 * Build example:
 *   gcc -std=c11 -Wall -Wextra -I../chassis_control/Inc path_plan_pc_test.c -o path_plan_pc_test
 *
 * Run examples:
 *   ./path_plan_pc_test
 *   ./path_plan_pc_test R2,R2,R1,R2,R2,R1,FAKE,NONE,R1,NONE,NONE,R1
 *
 * The custom map argument describes nodes 0..11. Nodes 12/13 remain NONE.
 */

static const char *kfs_type_name(KFS_Type type) {
    switch (type) {
        case KFS_NONE: return "NONE";
        case KFS_R1: return "R1";
        case KFS_R2: return "R2";
        case KFS_FAKE: return "FAKE";
        default: return "UNKNOWN";
    }
}

static int parse_kfs_type(const char *text, KFS_Type *type) {
    if (strcmp(text, "0") == 0 || strcmp(text, "NONE") == 0 || strcmp(text, "N") == 0) {
        *type = KFS_NONE;
        return 1;
    }
    if (strcmp(text, "1") == 0 || strcmp(text, "R1") == 0) {
        *type = KFS_R1;
        return 1;
    }
    if (strcmp(text, "2") == 0 || strcmp(text, "R2") == 0) {
        *type = KFS_R2;
        return 1;
    }
    if (strcmp(text, "3") == 0 || strcmp(text, "FAKE") == 0 || strcmp(text, "F") == 0) {
        *type = KFS_FAKE;
        return 1;
    }
    return 0;
}

static int load_map_from_arg(const char *arg, KFS_Type map[TOTAL_NODES]) {
    char buffer[256];
    int count = 0;

    strncpy(buffer, arg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *token = strtok(buffer, ",");
    while (token != NULL && count < GRID_NODES) {
        if (!parse_kfs_type(token, &map[count])) {
            printf("Invalid map token at node %d: %s\n", count, token);
            return 0;
        }
        count++;
        token = strtok(NULL, ",");
    }

    if (count != GRID_NODES || token != NULL) {
        printf("Map must contain exactly %d comma-separated nodes for 0..%d.\n",
               GRID_NODES,
               GRID_NODES - 1);
        return 0;
    }

    map[ENTRY_NODE] = KFS_NONE;
    map[EXIT_NODE] = KFS_NONE;
    return 1;
}

static void print_map(const KFS_Type map[TOTAL_NODES]) {
    printf("Map nodes 0..11:\n");
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            int node = r * COLS + c;
            printf("%2d:%-4s ", node, kfs_type_name(map[node]));
        }
        printf("\n");
    }
}

static void print_plan_summary(const PlanResult *plan) {
    printf("\n=== Plan Summary ===\n");
    printf("R2_TAKEN_COUNT: %d\n", R2_TAKEN_COUNT);

    printf("Take order: ");
    for (int i = 0; i < R2_TAKEN_COUNT; i++) {
        printf("%d ", plan->r2_taken[i]);
    }
    printf("\n");

    printf("Path length: %d\n", plan->path_len);
    printf("Path: ");
    for (int i = 0; i < plan->path_len; i++) {
        if (plan->path[i] == ENTRY_NODE) {
            printf("Entry ");
        } else if (plan->path[i] == EXIT_NODE) {
            printf("Exit ");
        } else {
            printf("%d ", plan->path[i]);
        }
    }
    printf("\n");
}

int main(int argc, char *argv[]) {
    KFS_Type map[TOTAL_NODES];
    memcpy(map, initial_map, sizeof(map));

    if (argc > 2) {
        printf("Usage:\n");
        printf("  %s\n", argv[0]);
        printf("  %s R2,R2,R1,R2,R2,R1,FAKE,NONE,R1,NONE,NONE,R1\n", argv[0]);
        return 1;
    }

    if (argc == 2 && !load_map_from_arg(argv[1], map)) {
        return 1;
    }

    print_map(map);
    PlanResult plan = plan_route(map);
    print_plan_summary(&plan);
    return 0;
}

#include "../chassis_control/Src/path_plan.c"

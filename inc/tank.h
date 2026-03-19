#ifndef __TANK_H__
#define __TANK_H__

#include "main.h"

#define TANK_FIRE_COOL_DOWN_MS 2000

typedef enum tank_dir
{
    TANK_DIR_UNKNOWN,
    TANK_DIR_UP,
    TANK_DIR_DOWN,
} tank_dir;

typedef struct tank_ctx
{
    uint16_t    id;

    tg_obj      tg_body;
    tg_obj      tg_missle;

    tank_dir    dir;

    // Track when last shot occured to handle cooldown
    uint64_t    fire_ts_ms;

    // Stats
    uint64_t    fired;
    uint64_t    hits;
    uint64_t    damage;
} tank_ctx;


void tank_obj_init(tg_ctx* ctx, tank_ctx* tank, uint16_t color);

void tank_shoot(tank_ctx* tank);

int tank_missle_collision(tank_ctx* tank, tg_obj* missle);

#endif

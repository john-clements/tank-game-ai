#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "unistd.h"

#include "tank.h"

#include "ncurses.h" // Only to be used for color macros

#define TANK_WIDTH      5
#define TANK_HEIGHT     2

#define MISSLE_WIDTH    1
#define MISSLE_HEIGHT   2

#define MISSLE_VELOCITY 2.0f

uint16_t g_tank_obj_id = 0;

void tank_obj_init(tg_ctx* ctx, tank_ctx* tank, uint16_t color)
{
    tank->id = g_tank_obj_id++;

    tg_obj_init(&tank->tg_body,     color,          TANK_WIDTH,     TANK_HEIGHT,    1.0f);
    tg_obj_init(&tank->tg_missle,   COLOR_WHITE,    MISSLE_WIDTH,   MISSLE_HEIGHT,  1.0f);

    tank->tg_body.on = 1;

    tank->tg_body.max_v_x = MAX_V;
    tank->tg_body.max_v_y = MAX_V;

    tank->tg_missle.max_v_x = MISSLE_VELOCITY;
    tank->tg_missle.max_v_y = MISSLE_VELOCITY;

    tg_ctx_add_obj(ctx, &tank->tg_body);
    tg_ctx_add_obj(ctx, &tank->tg_missle);
}

int tank_projectile_cool_down_ms(tank_ctx* tank)
{
    uint64_t time_ms = get_time_us() / 1000;

    if (time_ms - tank->fire_ts_ms < TANK_FIRE_COOL_DOWN_MS)
        return TANK_FIRE_COOL_DOWN_MS - (time_ms - tank->fire_ts_ms);

    return 0;
}

void tank_shoot(tank_ctx* tank)
{
    uint64_t time_ms = get_time_us() / 1000;

    if (time_ms - tank->fire_ts_ms < TANK_FIRE_COOL_DOWN_MS)
        return;

    tank->fire_ts_ms    = time_ms;
    tank->tg_missle.x   = tank->tg_body.x + TANK_WIDTH/2;
    tank->tg_missle.y   = tank->tg_body.y;

    if (tank->dir == TANK_DIR_UP)
        tank->tg_missle.v_y = -MISSLE_VELOCITY;
    else if (tank->dir == TANK_DIR_DOWN)
        tank->tg_missle.v_y = MISSLE_VELOCITY;

    tank->tg_missle.on = 1;

    tank->fired++;
}

int tank_missle_collision(tank_ctx* tank, tg_obj* missle)
{
    if ((missle->x + missle->width >= tank->tg_body.x) &&
        (missle->x <= tank->tg_body.x + tank->tg_body.width) &&
        (missle->y + missle->height >= tank->tg_body.y) &&
        (missle->y <= tank->tg_body.y + tank->tg_body.height))
    {
        return 1;
    }

    return 0;
}
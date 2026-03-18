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

#define MISSLE_VELOCITY 3

uint16_t g_tank_obj_id = 0;

void tank_obj_init(tg_ctx* ctx, tank_ctx* tank, uint16_t color)
{
    tank->id = g_tank_obj_id++;

    tg_obj_init(&tank->tg_body,     color,          TANK_WIDTH,     TANK_HEIGHT,    1.0f);
    tg_obj_init(&tank->tg_missle,   COLOR_WHITE,    MISSLE_WIDTH,   MISSLE_HEIGHT,  1.0f);

    tank->tg_body.on = 1;

    tg_ctx_add_obj(ctx, &tank->tg_body);
    tg_ctx_add_obj(ctx, &tank->tg_missle);
}

void tank_shoot(tank_ctx* tank)
{
    tank->tg_missle.x = tank->tg_body.x + TANK_WIDTH/2;
    tank->tg_missle.y = tank->tg_body.y;

    if (tank->dir == TANK_DIR_UP)
        tank->tg_missle.v_y = -MISSLE_VELOCITY;
    else if (tank->dir == TANK_DIR_DOWN)
        tank->tg_missle.v_y = MISSLE_VELOCITY;

    tank->tg_missle.on = 1;
}


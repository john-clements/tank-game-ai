#ifndef __TANK_H__
#define __TANK_H__

#include "main.h"


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
} tank_ctx;


void tank_obj_init(tg_ctx* ctx, tank_ctx* tank, uint16_t color);

#endif

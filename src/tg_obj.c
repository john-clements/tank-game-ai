#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "unistd.h"

#include "main.h"


uint16_t g_obj_id = 3;
void tg_obj_init(tg_obj* obj, uint16_t color, int width, int height, float mass)
{
    memset((void*)obj, 0, sizeof(tg_obj));

    obj->id     = g_obj_id++;
    obj->color  = color;

    obj->width  = width;
    obj->height = height;

    obj->mass   = mass;

    int max_y, max_x;
    get_screen_limits(&max_x, &max_y);

    obj->min_x_boundary = 1;
    obj->max_x_boundary = max_x;
    obj->min_y_boundary = 1;
    obj->max_y_boundary = max_y;
}

void tg_obj_pos_set_middle(tg_obj* obj)
{
    int max_y, max_x;
    get_screen_limits(&max_x, &max_y);
 
    obj->x = ((float)max_x - (float)obj->width) / 2.0f;
    obj->y = ((float)max_y - (float)obj->height) / 2.0f;
}

int tg_obj_process(tg_obj* obj)
{
    int collision = 0;

    if (!obj->on)
        return 0;

    obj->v_x = obj->v_x + obj->f_x / obj->mass;
    obj->v_y = obj->v_y + obj->f_y / obj->mass;

    obj->x = obj->x + obj->v_x;
    obj->y = obj->y + obj->v_y;

    // Collision detetion with boundaries

    // Horizontal
    if (obj->x + obj->width >= obj->max_x_boundary - 1)
    {
        obj->x = obj->max_x_boundary - 1 - obj->width;
        if (obj->elastic)
            obj->v_x = -obj->v_x;
        else
            obj->v_x = 0;

        collision = 1;
    }
    else if (obj->x <= obj->min_x_boundary)
    {
        obj->x = obj->min_x_boundary;
        if (obj->elastic)
            obj->v_x = -obj->v_x;
        else
            obj->v_x = 0;

        collision = 1;
    }

    // Vertical
    if (obj->y + obj->height >= obj->max_y_boundary)
    {
        obj->y = obj->max_y_boundary - obj->height;
        if (obj->elastic)
            obj->v_y = -obj->v_y;
        else
            obj->v_y = 0;

        collision = 1;
    }
    else if (obj->y <= obj->min_y_boundary)
    {
        obj->y = obj->min_y_boundary;
        if (obj->elastic)
            obj->v_y = -obj->v_y;
        else
            obj->v_y = 0;

        collision = 1;
    }

    if (obj->v_y > obj->max_v_y)
        obj->v_y = obj->max_v_y;
    else if (obj->v_y < -obj->max_v_y)
        obj->v_y = -obj->max_v_y;

    if (obj->v_x > obj->max_v_x)
        obj->v_x = obj->max_v_x;
    else if (obj->v_x < -obj->max_v_x)
        obj->v_x = -obj->max_v_x;

    return collision;
}

void tg_ctx_add_obj(tg_ctx* ctx, tg_obj* obj)
{
    if (ctx->obj_list_cnt >= MAX_OBJ_CNT)
    {
        // Return error here
        return;
    }

    ctx->obj_list[ctx->obj_list_cnt++] = obj;
}

////////////////////////////////
// DISTANCE FROM OBJECT EDGES //
////////////////////////////////

// distance X of b - a
float tg_obj_dist_x(tg_obj* a, tg_obj* b)
{
    if ((a->x + (float)a->width) < b->x)
        return b->x - (a->x + (float)a->width);
    else if (a->x > (b->x + (float)b->width))
        return (b->x + (float)b->width) - a->x;

    return 0.0f;
}

// distance Y of a - b
float tg_obj_dist_y(tg_obj* a, tg_obj* b)
{
    if ((a->y + (float)a->height) < b->y)
        return b->y - (a->y + (float)a->height);
    else if (a->x > (b->y + (float)b->height))
        return (b->y + (float)b->height) - a->y;

    return 0.0f;
}

//////////////////////////////////
// DISTANCE FROM OBJECT CENTERS //
//////////////////////////////////

// distance X of b - a
float tg_obj_dist_x_center(tg_obj* a, tg_obj* b)
{
    return (b->x + ((float)b->width)/2) - (a->x + ((float)a->width)/2.0f);
}

// distance Y of a - b
float tg_obj_dist_y_center(tg_obj* a, tg_obj* b)
{
    return (b->y + ((float)b->height)/2) - (a->y + ((float)a->height)/2.0f);
}
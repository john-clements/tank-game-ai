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

void tg_obj_process(tg_obj* obj)
{
    if (!obj->on)
        return;

    obj->v_x = obj->v_x + obj->f_x / obj->mass;
    obj->v_y = obj->v_y + obj->f_y / obj->mass;

    obj->x = obj->x + obj->v_x;
    obj->y = obj->y + obj->v_y;

    // Collision detetion with boundaries

    // Horizontal
    if (obj->x + obj->width >= obj->max_x_boundary - 1)
    {
        obj->x = obj->max_x_boundary - 1 - obj->width;
        obj->v_x = -obj->v_x;
    }
    else if (obj->x <= obj->min_x_boundary)
    {
        obj->x = obj->min_x_boundary;
        obj->v_x = -obj->v_x;
    }

    // Vertical
    if (obj->y + obj->height >= obj->max_y_boundary)
    {
        obj->y = obj->max_y_boundary - obj->height;
        if (obj->elastic)
            obj->v_y = -obj->v_y;
        else
            obj->v_y = 0;
    } else if (obj->y <= obj->min_y_boundary)
    {
        obj->y = obj->min_y_boundary;
        if (obj->elastic)
            obj->v_y = -obj->v_y;
        else
            obj->v_y = 0;
    }

    if (obj->v_y > MAX_V)
        obj->v_y = MAX_V;
    else if (obj->v_y < -MAX_V)
        obj->v_y = -MAX_V;

    if (obj->v_x > MAX_V)
        obj->v_x = MAX_V;
    else if (obj->v_x < -MAX_V)
        obj->v_x = -MAX_V;
}

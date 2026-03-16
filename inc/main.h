#ifndef __MAIN_H__
#define __MAIN_H__

#include "stdint.h"

#include "ddpg.h"

#define MAX_V 2.0
#define MAX_F 2.0

typedef struct tg_obj
{
    uint16_t id;

    uint16_t color;

    int width;
    int height;

    // Position
    float x;
    float y;

    // Velocity
    float v_x;
    float v_y;

    // Force
    float f_x;
    float f_y;

    float mass;

    // Handling
    int manual_process;
} tg_obj;

typedef struct tg_state
{
    // Configurable based on game design

    // Force vectors against obj
    float f_x;
    float f_y;

    // Obj AI processing
    DDPG*       ddpg;
    uint32_t    step;
    float       target_x;
    float       target_y;
    uint32_t    episode;
} tg_state;

typedef struct tg_ctx
{
    tg_obj*     obj_list;
    uint32_t    obj_list_cnt;

    tg_state    state;
} tg_ctx;

void tg_init();
void tg_start_engine(tg_ctx* ctx);

void get_screen_limits(int* x, int* y);

void tg_text_reset();
void tg_draw_text(int line, const char *fmt, ...);


void tg_obj_pos_set_middle(tg_obj* obj);
void tg_obj_init(tg_obj* obj, uint16_t color, int width, int height, float mass);
void tg_obj_process(tg_obj* obj);

// AI function prototypes
void init_tg_ai(tg_ctx* ctx);
void step_tg_ai(tg_ctx* ctx);
void free_tg_ai(tg_ctx* ctx);

void start_ai_obj_test();
void draw_tg_ai_status(tg_ctx* ctx);

#endif

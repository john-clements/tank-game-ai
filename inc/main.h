#ifndef __MAIN_H__
#define __MAIN_H__

#include "stdint.h"

#include "ddpg.h"

#define MAX_V 1.2
#define MAX_F 0.5

typedef struct tg_obj
{
    uint16_t id;

    uint16_t color;

    // Position
    float x;
    float y;

    // Velocity
    float v_x;
    float v_y;

    // Force
    float f_x;
    float f_y;

    // Physical properties
    int     width;
    int     height;
    float   mass;
    int     elastic;

    // Handling
    int manual_process;

    // Currently active
    int on;

    // Boundaries
    float min_x_boundary;
    float max_x_boundary;
    float min_y_boundary;
    float max_y_boundary;
} tg_obj;

typedef struct tg_ai_ctx
{
    DDPG*       ddpg;
    uint32_t    step;
    uint32_t    episode;
} tg_ai_ctx;

#define TANK_ACTION_CNT 3
typedef struct tg_state
{
    // Configurable based on game design

    // Force vectors against obj
    float f_x;
    float f_y;

    // Target destination
    float target_x;
    float target_y;

    // State time context
    float fire_decay;

    float reward[TANK_ACTION_CNT];

    // Obj AI processing
    tg_ai_ctx ai_ctx;
    tg_ai_ctx reward_classifier;
} tg_state;

#define MAX_OBJ_CNT     4
#define MAX_STATE_CNT   2
typedef struct tg_ctx
{
    tg_obj*     obj_list[MAX_OBJ_CNT];
    uint32_t    obj_list_cnt;

    tg_state    state[MAX_STATE_CNT];
} tg_ctx;

uint64_t get_time_us();

void tg_init();
void tg_start_engine(tg_ctx* ctx);
void tg_ctx_add_obj(tg_ctx* ctx, tg_obj* obj);
float tg_obj_dist_x(tg_obj* a, tg_obj* b);
float tg_obj_dist_y(tg_obj* a, tg_obj* b);
float tg_obj_dist_x_center(tg_obj* a, tg_obj* b);
float tg_obj_dist_y_center(tg_obj* a, tg_obj* b);

void get_screen_limits(int* x, int* y);

void tg_text_reset();
void tg_draw_text(int line, const char *fmt, ...);
void tg_text_set_col(int line, int col);


void tg_obj_pos_set_middle(tg_obj* obj);
void tg_obj_init(tg_obj* obj, uint16_t color, int width, int height, float mass);
int tg_obj_process(tg_obj* obj);

// AI function prototypes
void init_tg_ai(tg_ctx* ctx);
void step_tg_ai(tg_ctx* ctx);
void free_tg_ai(tg_ctx* ctx);

void start_ai_obj_test();
void draw_tg_ai_status(tg_ctx* ctx);

#endif

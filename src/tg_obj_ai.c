#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "ncurses.h"
#include "unistd.h"
#include "math.h"

#include "tank.h"

#define ACTION_MAGNITUDE_EN

#define EPISODE_LENGTH  200
#define LAYER_SIZE      2
#define REPLAY_BUF_SIZE 20000
#define BATCH_SIZE      64
#define REWARD_SIZE     2
#define STATE_SIZE      6
#define ACTION_SIZE     2
#define STEP_CONTROL    0.02f

#define TANK_LOWER_ID   0
#define TANK_UPPER_ID   1

tank_ctx g_tank_lower = {0};
tank_ctx g_tank_upper = {0};

void init_tg_ai(tg_ctx* ctx)
{
    int layers[LAYER_SIZE]  = {128, 64};

    for (int i = 0; i < MAX_STATE_CNT; i++)
    {
        tg_ai_ctx* ai_ctx = &ctx->state[i].ai_ctx;

        DDPG* ddpg = ddpg_create(STATE_SIZE, ACTION_SIZE, NULL, LAYER_SIZE, layers, LAYER_SIZE, layers, REPLAY_BUF_SIZE, BATCH_SIZE, REWARD_SIZE);

        ai_ctx->ddpg = ddpg;
        ai_ctx->step = 0;
    }

    ctx->state[TANK_LOWER_ID].target_x = ((float)g_tank_lower.tg_body.max_x_boundary - (float)g_tank_lower.tg_body.width) / 2.0f;
    ctx->state[TANK_LOWER_ID].target_y = g_tank_lower.tg_body.y;

    ctx->state[TANK_UPPER_ID].target_x = ((float)g_tank_upper.tg_body.max_x_boundary - (float)g_tank_upper.tg_body.width) / 2.0f;
    ctx->state[TANK_UPPER_ID].target_y = g_tank_upper.tg_body.y;
}

void free_tg_ai(tg_ctx* ctx)
{
    for (int i = 0; i < MAX_STATE_CNT; i++)
        ddpg_destroy(ctx->state[i].ai_ctx.ddpg);
}

#define ACTION_UP   0
#define ACTION_DOWN 1
#define ACTION_IDLE 2

int get_action_index(float action)
{
    if (action > 0.33f)
        return ACTION_UP;
    else if (action < -0.33f)
        return ACTION_DOWN;

    return ACTION_IDLE;
}

void process_action(float action, float* param)
{
#ifdef ACTION_MAGNITUDE_EN
    *param = action;
#else
    float action_scale = get_action_index(action);

    if (action_scale == ACTION_UP)
        *param = *param + STEP_CONTROL;
    else if (action_scale == ACTION_DOWN)
        *param = *param - STEP_CONTROL;

    if (*param > MAX_F)
        *param = MAX_F;
    if (*param < -MAX_F)
        *param = -MAX_F;
#endif
}

void get_reward(tg_state* state_ctx, tg_obj* obj, float* reward)
{
    reward[0] = -fabs(obj->x - state_ctx->target_x) / state_ctx->target_x;
    reward[1] = -fabs(obj->y - state_ctx->target_y) / state_ctx->target_y;
}

void state_step(tg_state* state_ctx, tg_obj* obj, float* reward, float* action)
{
    process_action(action[0], &obj->f_x);
    process_action(action[1], &obj->f_y);

    get_reward(state_ctx, obj, reward);

    tg_obj_process(obj);
}

float random_target()
{
    float range = 2.0f / STEP_CONTROL;

    int x = deepc_random_int(0, (int)range);

    return STEP_CONTROL * (float)x - 1.0f;
}

float feature_normalize(float val, float min, float max)
{
    return (val - min) / (max - min);
}

// 0 -> x pos
// 1 -> y pos
// 2 -> x velocity
// 3 -> y velocity
// 4 -> x force
// 5 -> y force
void get_state(tg_state* state_ctx, tg_obj* obj, float* state)
{
    state[0] = feature_normalize(obj->x, 0, state_ctx->target_x*2);
    state[1] = feature_normalize(obj->y, 0, state_ctx->target_y*2);
    state[2] = feature_normalize(obj->v_x, -MAX_V, MAX_V);
    state[3] = feature_normalize(obj->v_y, -MAX_V, MAX_V);
    state[4] = feature_normalize(obj->f_x, -MAX_F, MAX_F);
    state[5] = feature_normalize(obj->f_y, -MAX_F, MAX_F);
}

void step_tg_ai_tank(tg_state* state_ctx, tg_obj* tank)
{
    float reward[REWARD_SIZE];
    float state[STATE_SIZE];

    tg_ai_ctx* ai_ctx = &state_ctx->ai_ctx;

    if (ai_ctx->step == 0)
    {
        ddpg_new_episode(ai_ctx->ddpg);

        tank->f_x = random_target();
        tank->f_y = random_target();
    }

    get_state(state_ctx, tank, state);

    float* action = ddpg_action(ai_ctx->ddpg, state);

    state_step(state_ctx, tank, reward, action);

    get_state(state_ctx, tank, state);

    ai_ctx->step++;
    if (ai_ctx->step >= EPISODE_LENGTH)
    {
        ai_ctx->step = 0;
        ai_ctx->episode++;
    }

    if (ai_ctx->step == 0)
        ddpg_observe(ai_ctx->ddpg, action, reward, state, 1);
    else
        ddpg_observe(ai_ctx->ddpg, action, reward, state, 0);

    ddpg_train(ai_ctx->ddpg, 0.99);

    ddpg_soft_update_target_networks(ai_ctx->ddpg, .005);
}

void proccess_tank_missle(tank_ctx* tank)
{
    if (!tank->tg_missle.on)
    {
        if (deepc_random_int(0, 10) == 5)
            tank_shoot(tank);
        else
            return;
    }

    if (tg_obj_process(&tank->tg_missle))
    {
        // Collision
        tank->tg_missle.on = 0;
    }

    if (tank->tg_missle.on)
    {
        tank_ctx* tank_opposite = NULL;

        if (tank->id == TANK_LOWER_ID)
            tank_opposite = &g_tank_upper;
        else if (tank->id == TANK_UPPER_ID)
            tank_opposite = &g_tank_lower;

        if (tank_missle_collision(tank_opposite, &tank->tg_missle))
        {
            // Tank collision
            tank->tg_missle.on = 0;
            tank->hits++;
            tank_opposite->damage++;
        }
    }
}

void step_tg_ai(tg_ctx* ctx)
{
    step_tg_ai_tank(&ctx->state[TANK_LOWER_ID], &g_tank_lower.tg_body);
    step_tg_ai_tank(&ctx->state[TANK_UPPER_ID], &g_tank_upper.tg_body);

    proccess_tank_missle(&g_tank_lower);
    proccess_tank_missle(&g_tank_upper);
}

void tank_init_upper(tg_ctx* ctx, tank_ctx* tank)
{
    tank_obj_init(ctx, tank, COLOR_MAGENTA);

    tank->dir = TANK_DIR_DOWN;

    tank->tg_body.max_y_boundary = tank->tg_body.max_y_boundary / 2;

    tg_obj_pos_set_middle(&tank->tg_body);

    tank->tg_body.y = tank->tg_body.y - tank->tg_body.y / 2;
}

void tank_init_lower(tg_ctx* ctx, tank_ctx* tank)
{
    tank_obj_init(ctx, tank, COLOR_GREEN);

    tank->dir = TANK_DIR_UP;

    tank->tg_body.min_y_boundary = tank->tg_body.max_y_boundary / 2;

    tg_obj_pos_set_middle(&tank->tg_body);

    tank->tg_body.y = tank->tg_body.y + tank->tg_body.y / 2;
}

void start_ai_obj_test()
{
    tg_ctx ctx = {0};

    tg_init();

    tank_init_lower(&ctx, &g_tank_lower);
    tank_init_upper(&ctx, &g_tank_upper);

    g_tank_lower.tg_body.manual_process = 1;
    g_tank_upper.tg_body.manual_process = 1;

    g_tank_lower.tg_missle.manual_process = 1;
    g_tank_upper.tg_missle.manual_process = 1;

    tg_start_engine(&ctx);
}

void draw_tg_ai_status_tank(tg_state* state_ctx, tank_ctx* tank)
{
    float reward[REWARD_SIZE];

    get_reward(state_ctx, &tank->tg_body, reward);

    tg_draw_text(0, "Episode  : %d -> Fx=%.2f  Fy=%.2f", state_ctx->ai_ctx.episode, tank->tg_body.f_x, tank->tg_body.f_y);
    tg_draw_text(1, "Position : X=%-.3f  Y=%-.3f", tank->tg_body.x, tank->tg_body.y);

    tg_draw_text(2, "Reward   : [");
    for (int i = 0; i < REWARD_SIZE - 1; i++)
        tg_draw_text(2, "%-.3f, ", reward[i]);
    tg_draw_text(2, "%-.3f]", reward[REWARD_SIZE - 1]);
}

void draw_tg_ai_status(tg_ctx* ctx)
{
    tg_ai_ctx* ai_ctx = &ctx->state[0].ai_ctx;

    tg_text_reset();

    draw_tg_ai_status_tank(&ctx->state[TANK_LOWER_ID], &g_tank_lower);

    tg_text_set_col(0, 40);
    tg_text_set_col(1, 40);
    tg_text_set_col(2, 40);

    draw_tg_ai_status_tank(&ctx->state[TANK_UPPER_ID], &g_tank_upper);
}

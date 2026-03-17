#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "ncurses.h"
#include "unistd.h"
#include "math.h"

#include "main.h"

#define ACTION_MAGNITUDE_EN

#define EPISODE_LENGTH  200
#define LAYER_SIZE      2
#define REPLAY_BUF_SIZE 20000
#define BATCH_SIZE      64
#define REWARD_SIZE     2
#define STATE_SIZE      6
#define ACTION_SIZE     2
#define STEP_CONTROL    0.02f

#define SQUARE_IDX      0

void init_tg_ai(tg_ctx* ctx)
{
    int layers[LAYER_SIZE]  = {128, 64};

    tg_ai_ctx* ai_ctx = &ctx->state.ai_ctx;

    DDPG* ddpg = ddpg_create(STATE_SIZE, ACTION_SIZE, NULL, LAYER_SIZE, layers, LAYER_SIZE, layers, REPLAY_BUF_SIZE, BATCH_SIZE, REWARD_SIZE);

    ai_ctx->ddpg = ddpg;
    ai_ctx->step = 0;

    int max_y, max_x;
    get_screen_limits(&max_x, &max_y);

    ctx->state.target_x = ((float)max_x - (float)ctx->obj_list[SQUARE_IDX].width) / 2.0f;
    ctx->state.target_y = ((float)max_y - (float)ctx->obj_list[SQUARE_IDX].height) / 2.0f;
}

void free_tg_ai(tg_ctx* ctx)
{
    ddpg_destroy(ctx->state.ai_ctx.ddpg);
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

void get_reward(tg_ctx* ctx, float* reward)
{
    tg_obj* obj = &ctx->obj_list[SQUARE_IDX];

    reward[0] = -fabs(obj->x - ctx->state.target_x) / ctx->state.target_x;
    reward[1] = -fabs(obj->y - ctx->state.target_y) / ctx->state.target_y;
}

void state_step(tg_ctx* ctx, float* reward, float* action)
{
    tg_obj* obj = &ctx->obj_list[SQUARE_IDX];

    process_action(action[0], &obj->f_x);
    process_action(action[1], &obj->f_y);

    get_reward(ctx, reward);

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
void get_state(tg_ctx* ctx, float* state)
{
    tg_obj* obj = &ctx->obj_list[SQUARE_IDX];

    state[0] = feature_normalize(obj->x, 0, ctx->state.target_x*2);
    state[1] = feature_normalize(obj->y, 0, ctx->state.target_y*2);
    state[2] = feature_normalize(obj->v_x, -MAX_V, MAX_V);
    state[3] = feature_normalize(obj->v_y, -MAX_V, MAX_V);
    state[4] = feature_normalize(obj->f_x, -MAX_F, MAX_F);
    state[5] = feature_normalize(obj->f_y, -MAX_F, MAX_F);
}

void step_tg_ai(tg_ctx* ctx)
{
    float reward[REWARD_SIZE];
    float state[STATE_SIZE];

    tg_ai_ctx* ai_ctx = &ctx->state.ai_ctx;

    if (ai_ctx->step == 0)
    {
        ddpg_new_episode(ai_ctx->ddpg);

        ctx->obj_list[SQUARE_IDX].f_x = random_target();
        ctx->obj_list[SQUARE_IDX].f_y = random_target();
    }

    get_state(ctx, state);

    float* action = ddpg_action(ai_ctx->ddpg, state);

    state_step(ctx, reward, action);

    get_state(ctx, state);

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

#define OBJ_CNT 1
void start_ai_obj_test()
{
    tg_ctx ctx = {0};

    tg_obj obj[OBJ_CNT] = {0};

    tg_init();

    memset((void*)obj, 0, sizeof(tg_obj)*OBJ_CNT);

    obj[SQUARE_IDX].id = 2;
    obj[SQUARE_IDX].color = COLOR_GREEN;

    obj[SQUARE_IDX].width = 4;
    obj[SQUARE_IDX].height = 2;

    obj[SQUARE_IDX].mass = 1;

    obj[SQUARE_IDX].manual_process = 1;

    tg_obj_init(&obj[SQUARE_IDX], COLOR_GREEN, 4, 2, 1);

    tg_obj_pos_set_middle(&obj[SQUARE_IDX]);

    ctx.obj_list        = obj;
    ctx.obj_list_cnt    = OBJ_CNT;

    tg_start_engine(&ctx);
}

void draw_tg_ai_status(tg_ctx* ctx)
{
    tg_ai_ctx* ai_ctx = &ctx->state.ai_ctx;

    float reward[REWARD_SIZE];

    get_reward(ctx, reward);

    tg_text_reset();

    tg_draw_text(0, "Episode  : %d -> Fx=%.2f  Fy=%.2f", ai_ctx->episode, ctx->obj_list[SQUARE_IDX].f_x, ctx->obj_list[SQUARE_IDX].f_y);
    tg_draw_text(1, "Position : X=%-3f  Y=%-3f", ctx->obj_list[SQUARE_IDX].x, ctx->obj_list[SQUARE_IDX].y);

    tg_draw_text(2, "Reward   : [");
    for (int i = 0; i < REWARD_SIZE - 1; i++)
        tg_draw_text(2, "%.3f, ", reward[i]);
    tg_draw_text(2, "%.3f]", reward[REWARD_SIZE - 1]);
}
